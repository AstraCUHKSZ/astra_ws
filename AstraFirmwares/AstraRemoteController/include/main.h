#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>

#ifndef REMOTE_SIDE
#define REMOTE_SIDE 0
#endif

#define REMOTE_DEVICE_ID 0x01

#define REMOTE_SIDE_LEFT 0x00
#define REMOTE_SIDE_RIGHT 0x01

#define FORCE_SENSOR_PIN 4

#define BUTTON0_DRIVE_PIN 3
#define BUTTON0_READ_PIN 2

#define BUTTON1_DRIVE_PIN 1
#define BUTTON1_READ_PIN 0

#define REMOTE_SEND_PERIOD_MS 20
#define BUTTON_DEBOUNCE_MS 30
#define BUTTON_DOUBLE_CLICK_MS 300

typedef enum {
  REMOTE_EVENT_NONE = 0x00,
  REMOTE_EVENT_SINGLE_PRESS = 0x01,
  REMOTE_EVENT_DOUBLE_PRESS = 0x02,
} remote_event_t;

#endif
