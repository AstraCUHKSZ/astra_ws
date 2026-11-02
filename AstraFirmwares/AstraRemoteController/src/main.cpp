#include <Arduino.h>
#include <string.h>
#include <lwip/sockets.h>
#include "main.h"
#include "comm.h"

struct ButtonState {
  uint8_t drive_pin;
  uint8_t read_pin;
  bool last_raw_pressed;
  bool stable_pressed;
  bool pending_single;
  uint32_t last_raw_change_ms;
  uint32_t single_due_ms;
};

struct ButtonEvent {
  uint8_t event_code;
  uint8_t button_id;
};

static ButtonState buttons[] = {
  {BUTTON0_DRIVE_PIN, BUTTON0_READ_PIN, false, false, false, 0, 0},
  {BUTTON1_DRIVE_PIN, BUTTON1_READ_PIN, false, false, false, 0, 0},
};

static ButtonEvent event_queue[8];
static uint8_t event_queue_head = 0;
static uint8_t event_queue_tail = 0;

static void push_event(uint8_t event_code, uint8_t button_id) {
  uint8_t next_tail = (event_queue_tail + 1) % (sizeof(event_queue) / sizeof(event_queue[0]));
  if (next_tail == event_queue_head) {
    return;
  }

  event_queue[event_queue_tail] = {event_code, button_id};
  event_queue_tail = next_tail;
}

static ButtonEvent pop_event() {
  if (event_queue_head == event_queue_tail) {
    return {REMOTE_EVENT_NONE, 0};
  }

  ButtonEvent event = event_queue[event_queue_head];
  event_queue_head = (event_queue_head + 1) % (sizeof(event_queue) / sizeof(event_queue[0]));
  return event;
}

static void update_button(uint8_t button_id) {
  ButtonState *button = &buttons[button_id];
  uint32_t now = millis();
  bool raw_pressed = digitalRead(button->read_pin) == LOW;

  if (raw_pressed != button->last_raw_pressed) {
    button->last_raw_pressed = raw_pressed;
    button->last_raw_change_ms = now;
  }

  if (now - button->last_raw_change_ms >= BUTTON_DEBOUNCE_MS &&
      raw_pressed != button->stable_pressed) {
    button->stable_pressed = raw_pressed;

    if (!button->stable_pressed) {
      if (button->pending_single && now <= button->single_due_ms) {
        button->pending_single = false;
        push_event(REMOTE_EVENT_DOUBLE_PRESS, button_id);
      } else {
        button->pending_single = true;
        button->single_due_ms = now + BUTTON_DOUBLE_CLICK_MS;
      }
    }
  }

  if (button->pending_single && now > button->single_due_ms) {
    button->pending_single = false;
    push_event(REMOTE_EVENT_SINGLE_PRESS, button_id);
  }
}

static void send_remote_feedback(uint16_t force_raw, ButtonEvent event) {
  uint8_t payload[16] = {0};
  uint16_t force_network_order = htons(force_raw);

  payload[0] = REMOTE_DEVICE_ID;
  payload[1] = (uint8_t)REMOTE_SIDE;
  payload[2] = event.event_code;
  payload[3] = event.button_id;
  memcpy(payload + 4, &force_network_order, sizeof(force_network_order));

  comm_send_blocking(COMM_TYPE_FEEDBACK, payload);
}

void setup() {
  analogReadResolution(12);

  pinMode(FORCE_SENSOR_PIN, INPUT);
  pinMode(BUTTON0_READ_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_READ_PIN, INPUT_PULLUP);
  pinMode(BUTTON0_DRIVE_PIN, OUTPUT);
  pinMode(BUTTON1_DRIVE_PIN, OUTPUT);

  digitalWrite(BUTTON0_DRIVE_PIN, LOW);
  digitalWrite(BUTTON1_DRIVE_PIN, LOW);

  comm_init();
}

void loop() {
  static uint32_t last_send_ms = 0;
  uint32_t now = millis();

  update_button(0);
  update_button(1);

  if (now - last_send_ms < REMOTE_SEND_PERIOD_MS) {
    return;
  }
  last_send_ms = now;

  uint16_t force_raw = analogRead(FORCE_SENSOR_PIN);
  ButtonEvent event = pop_event();

  send_remote_feedback(force_raw, event);
}
