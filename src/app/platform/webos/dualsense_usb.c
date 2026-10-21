#include "dualsense_usb.h"

#include "logging.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <SDL.h>
#include <Limelight.h>

/* Sony DualSense / DualSense Edge */
#define DS_VENDOR 0x054cu
#define DS_PID_DS5 0x0ce6u
#define DS_PID_EDGE 0x0df2u
#define BUS_USB 0x03u
#define BUS_BLUETOOTH 0x05u

/* Hardcode ioctl numbers — webOS NDK may lack linux/hidraw.h. */
#define HIDIOCGRAWINFO ((2U << 30) | (8U << 16) | ((unsigned) ('H') << 8) | 0x03U)
#define HIDIOCGRAWUNIQ(len) ((2U << 30) | ((unsigned) (len) << 16) | ((unsigned) ('H') << 8) | 0x08U)

#define DS_USB_REPORT_LEN 63
#define DS_COMMON_OFF 1
#define DS_COMMON_LEN 47
#define DS_EFFECT_LEN 11

/*
 * Bluetooth output report 0x31 in SDL's HIDAPI layout: 0x02 header, common block, padding, CRC32.
 * Unlike the kernel's sequence-tagged layout, it carries no sequence number, so it cannot collide
 * with the 0x31 reports hid-playstation sends to the same pad for rumble.
 */
#define DS_BT_REPORT_LEN 78
#define DS_BT_COMMON_OFF 2
#define DS_BT_HEADER 0x02u
#define DS_BT_CRC_SEED 0xa2u

#define FLAG0_RIGHT_TRIGGER 0x04u
#define FLAG0_LEFT_TRIGGER 0x08u
#define FLAG1_MIC_LED 0x01u
#define FLAG1_LIGHTBAR 0x04u
#define FLAG1_PLAYER_LEDS 0x10u

#define OFF_MIC_LED 8
#define OFF_RIGHT_TRIGGER 10
#define OFF_LEFT_TRIGGER 21
#define OFF_PLAYER_LEDS 43
#define OFF_LIGHTBAR_RED 44

#define MAX_HIDRAW_NODES 10

struct hidraw_devinfo {
    uint32_t bustype;
    int16_t vendor;
    int16_t product;
};

struct dualsense_usb_t {
    int fd;
    char path[32];
    bool bluetooth;
    bool triggers_owned;
    bool lightbar_owned;
    bool player_led_owned;
    bool mic_led_owned;
    uint8_t left_effect[DS_EFFECT_LEN];
    uint8_t right_effect[DS_EFFECT_LEN];
    uint8_t lightbar[3];
    uint8_t player_leds;
    uint8_t mic_led;
    bool write_fail_logged;
};

static void normalize_mac(char *dst, size_t dst_len, const char *src) {
    size_t o = 0;
    if (src == NULL || dst_len == 0) {
        if (dst_len > 0) {
            dst[0] = '\0';
        }
        return;
    }
    for (const char *p = src; *p && o + 1 < dst_len; p++) {
        char c = *p;
        if (c == '-' || c == ':') {
            continue;
        }
        if (c >= 'A' && c <= 'F') {
            c = (char) (c - 'A' + 'a');
        }
        dst[o++] = c;
    }
    dst[o] = '\0';
}

static bool hidraw_read_uniq(int fd, char *out, size_t out_len) {
    char buf[128];
    memset(buf, 0, sizeof(buf));
    if (ioctl(fd, HIDIOCGRAWUNIQ(sizeof(buf)), buf) < 0) {
        return false;
    }
    normalize_mac(out, out_len, buf);
    return out[0] != '\0';
}

/* Returns BUS_USB or BUS_BLUETOOTH for a DualSense node, 0 for anything else. */
static uint32_t hidraw_dualsense_bus(int fd) {
    struct hidraw_devinfo info;
    memset(&info, 0, sizeof(info));
    if (ioctl(fd, HIDIOCGRAWINFO, &info) < 0) {
        return 0;
    }
    if (info.bustype != BUS_USB && info.bustype != BUS_BLUETOOTH) {
        return 0;
    }
    uint16_t vendor = (uint16_t) info.vendor;
    uint16_t product = (uint16_t) info.product;
    if (vendor != DS_VENDOR || (product != DS_PID_DS5 && product != DS_PID_EDGE)) {
        return 0;
    }
    return info.bustype;
}

/* webOS hands Bluetooth pads to the kernel through uhid, so they get hidraw nodes like USB pads. */
static int open_dualsense_node(const char *path, bool *bluetooth) {
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    uint32_t bus = hidraw_dualsense_bus(fd);
    if (bus == 0) {
        close(fd);
        return -1;
    }
    *bluetooth = bus == BUS_BLUETOOTH;
    return fd;
}

static bool serials_match(const char *a, const char *b) {
    char na[32], nb[32];
    normalize_mac(na, sizeof(na), a);
    normalize_mac(nb, sizeof(nb), b);
    return na[0] != '\0' && strcmp(na, nb) == 0;
}

static void fill_common(uint8_t *c, const dualsense_usb_t *ds) {
    memset(c, 0, DS_COMMON_LEN);
    if (ds->triggers_owned) {
        c[0] = (uint8_t) (FLAG0_RIGHT_TRIGGER | FLAG0_LEFT_TRIGGER);
        memcpy(c + OFF_RIGHT_TRIGGER, ds->right_effect, DS_EFFECT_LEN);
        memcpy(c + OFF_LEFT_TRIGGER, ds->left_effect, DS_EFFECT_LEN);
    }
    if (ds->mic_led_owned) {
        c[1] |= FLAG1_MIC_LED;
        c[OFF_MIC_LED] = ds->mic_led;
    }
    if (ds->lightbar_owned) {
        c[1] |= FLAG1_LIGHTBAR;
        c[OFF_LIGHTBAR_RED] = ds->lightbar[0];
        c[OFF_LIGHTBAR_RED + 1] = ds->lightbar[1];
        c[OFF_LIGHTBAR_RED + 2] = ds->lightbar[2];
    }
    if (ds->player_led_owned) {
        c[1] |= FLAG1_PLAYER_LEDS;
        c[OFF_PLAYER_LEDS] = (uint8_t) (ds->player_leds & 0x1Fu);
    }
}

static size_t build_bt_report(uint8_t *report, const dualsense_usb_t *ds) {
    memset(report, 0, DS_BT_REPORT_LEN);
    report[0] = 0x31;
    report[1] = DS_BT_HEADER;
    fill_common(report + DS_BT_COMMON_OFF, ds);

    /* The pad drops Bluetooth output reports whose CRC32 (seeded with the 0xA2 HID header) is wrong. */
    const uint8_t seed = DS_BT_CRC_SEED;
    Uint32 crc = SDL_crc32(0, &seed, 1);
    crc = SDL_crc32(crc, report, DS_BT_REPORT_LEN - 4);
    report[DS_BT_REPORT_LEN - 4] = (uint8_t) crc;
    report[DS_BT_REPORT_LEN - 3] = (uint8_t) (crc >> 8);
    report[DS_BT_REPORT_LEN - 2] = (uint8_t) (crc >> 16);
    report[DS_BT_REPORT_LEN - 1] = (uint8_t) (crc >> 24);
    return DS_BT_REPORT_LEN;
}

static bool flush_report(dualsense_usb_t *ds) {
    uint8_t report[DS_BT_REPORT_LEN];
    size_t len;
    if (ds->bluetooth) {
        len = build_bt_report(report, ds);
    } else {
        memset(report, 0, DS_USB_REPORT_LEN);
        report[0] = 0x02;
        fill_common(report + DS_COMMON_OFF, ds);
        len = DS_USB_REPORT_LEN;
    }

    ssize_t n = write(ds->fd, report, len);
    if (n != (ssize_t) len) {
        if (!ds->write_fail_logged) {
            commons_log_warn("Input", "DualSense %s write to %s failed: %s (further errors quiet)",
                             ds->bluetooth ? "Bluetooth" : "USB", ds->path,
                             n < 0 ? strerror(errno) : "short write");
            ds->write_fail_logged = true;
        }
        return false;
    }
    ds->write_fail_logged = false;
    return true;
}

dualsense_usb_t *dualsense_usb_open(SDL_GameController *controller) {
    if (controller == NULL) {
        return NULL;
    }
#if SDL_VERSION_ATLEAST(2, 0, 16)
    if (SDL_GameControllerGetType(controller) != SDL_CONTROLLER_TYPE_PS5) {
        return NULL;
    }
#else
    return NULL;
#endif

    SDL_Joystick *js = SDL_GameControllerGetJoystick(controller);
    const char *serial = NULL;
#if SDL_VERSION_ATLEAST(2, 0, 14)
    serial = js != NULL ? SDL_JoystickGetSerial(js) : NULL;
#endif

    int fd = -1;
    bool bluetooth = false;
    char path[32] = {0};

#if SDL_VERSION_ATLEAST(2, 0, 18)
    const char *sdl_path = SDL_GameControllerPath(controller);
    if (sdl_path != NULL && strncmp(sdl_path, "/dev/hidraw", 11) == 0) {
        fd = open_dualsense_node(sdl_path, &bluetooth);
        if (fd >= 0) {
            snprintf(path, sizeof(path), "%s", sdl_path);
        }
    }
#endif

    if (fd < 0) {
        int fallback_fd = -1;
        bool fallback_bluetooth = false;
        char fallback_path[32] = {0};
        for (int i = 0; i < MAX_HIDRAW_NODES; i++) {
            char candidate[32];
            bool try_bluetooth = false;
            snprintf(candidate, sizeof(candidate), "/dev/hidraw%d", i);
            int try_fd = open_dualsense_node(candidate, &try_bluetooth);
            if (try_fd < 0) {
                continue;
            }
            if (serial != NULL && serial[0] != '\0') {
                char uniq[32];
                if (hidraw_read_uniq(try_fd, uniq, sizeof(uniq)) && serials_match(uniq, serial)) {
                    fd = try_fd;
                    bluetooth = try_bluetooth;
                    snprintf(path, sizeof(path), "%s", candidate);
                    if (fallback_fd >= 0) {
                        close(fallback_fd);
                        fallback_fd = -1;
                    }
                    break;
                }
                if (fallback_fd < 0) {
                    fallback_fd = try_fd;
                    fallback_bluetooth = try_bluetooth;
                    snprintf(fallback_path, sizeof(fallback_path), "%s", candidate);
                } else {
                    close(try_fd);
                }
                continue;
            }
            fd = try_fd;
            bluetooth = try_bluetooth;
            snprintf(path, sizeof(path), "%s", candidate);
            break;
        }
        if (fd < 0 && fallback_fd >= 0) {
            fd = fallback_fd;
            bluetooth = fallback_bluetooth;
            snprintf(path, sizeof(path), "%s", fallback_path);
        } else if (fallback_fd >= 0) {
            close(fallback_fd);
        }
    }

    if (fd < 0) {
        commons_log_info("Input", "DualSense hidraw unavailable (no accessible node)");
        return NULL;
    }

    dualsense_usb_t *ds = calloc(1, sizeof(*ds));
    if (ds == NULL) {
        close(fd);
        return NULL;
    }
    ds->fd = fd;
    ds->bluetooth = bluetooth;
    snprintf(ds->path, sizeof(ds->path), "%s", path);
    commons_log_info("Input", "DualSense %s feedback on %s", bluetooth ? "Bluetooth" : "USB", ds->path);
    return ds;
}

void dualsense_usb_close(dualsense_usb_t *ds) {
    if (ds == NULL) {
        return;
    }
    /* Release trigger resistance so the pad is not left stiff after disconnect. */
    ds->triggers_owned = true;
    memset(ds->left_effect, 0, sizeof(ds->left_effect));
    memset(ds->right_effect, 0, sizeof(ds->right_effect));
    (void) flush_report(ds);
    close(ds->fd);
    free(ds);
}

bool dualsense_usb_set_adaptive_triggers(dualsense_usb_t *ds, uint8_t eventFlags,
                                         uint8_t typeLeft, uint8_t typeRight,
                                         const uint8_t *left, const uint8_t *right) {
    if (ds == NULL) {
        return false;
    }
    if (eventFlags & DS_EFFECT_RIGHT_TRIGGER) {
        ds->right_effect[0] = typeRight;
        if (right != NULL) {
            memcpy(ds->right_effect + 1, right, DS_EFFECT_PAYLOAD_SIZE);
        }
    }
    if (eventFlags & DS_EFFECT_LEFT_TRIGGER) {
        ds->left_effect[0] = typeLeft;
        if (left != NULL) {
            memcpy(ds->left_effect + 1, left, DS_EFFECT_PAYLOAD_SIZE);
        }
    }
    ds->triggers_owned = true;
    return flush_report(ds);
}

bool dualsense_usb_set_lightbar(dualsense_usb_t *ds, uint8_t r, uint8_t g, uint8_t b) {
    if (ds == NULL) {
        return false;
    }
    ds->lightbar[0] = r;
    ds->lightbar[1] = g;
    ds->lightbar[2] = b;
    ds->lightbar_owned = true;
    return flush_report(ds);
}

bool dualsense_usb_set_player_led(dualsense_usb_t *ds, uint8_t ledValue) {
    if (ds == NULL) {
        return false;
    }
    ds->player_leds = ledValue;
    ds->player_led_owned = true;
    return flush_report(ds);
}

bool dualsense_usb_set_mic_led(dualsense_usb_t *ds, uint8_t ledState) {
    if (ds == NULL) {
        return false;
    }
    ds->mic_led = ledState;
    ds->mic_led_owned = true;
    return flush_report(ds);
}
