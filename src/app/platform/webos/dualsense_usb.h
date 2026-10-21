#pragma once

/**
 * DualSense feedback on webOS via /dev/hidraw* (USB and Bluetooth).
 *
 * Host (Vibepollo/Sunshine) still sends Moonlight 0x5503/LED packets; this module
 * applies them as full output reports: 0x02 on USB, 0x31 with CRC32 on Bluetooth.
 * webOS passes Bluetooth pads to the kernel through uhid, so they expose hidraw too.
 * Rumble stays on SDL/EV_FF — reports never set the compatible-vibration valid flag.
 */

#include <stdbool.h>
#include <stdint.h>

#include <SDL_gamecontroller.h>

typedef struct dualsense_usb_t dualsense_usb_t;

/** Open hidraw for this PS5 pad (USB or Bluetooth); NULL if no accessible node. */
dualsense_usb_t *dualsense_usb_open(SDL_GameController *controller);

void dualsense_usb_close(dualsense_usb_t *ds);

bool dualsense_usb_set_adaptive_triggers(dualsense_usb_t *ds, uint8_t eventFlags,
                                         uint8_t typeLeft, uint8_t typeRight,
                                         const uint8_t *left, const uint8_t *right);

bool dualsense_usb_set_lightbar(dualsense_usb_t *ds, uint8_t r, uint8_t g, uint8_t b);

bool dualsense_usb_set_player_led(dualsense_usb_t *ds, uint8_t ledValue);

bool dualsense_usb_set_mic_led(dualsense_usb_t *ds, uint8_t ledState);
