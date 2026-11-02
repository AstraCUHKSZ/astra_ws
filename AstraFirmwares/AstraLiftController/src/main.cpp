#include <AccelStepper.h>
#include <Arduino.h>
#include <comm.h>
#include <lwip/sockets.h>

// #define STEPPER_CORRECT_DIR(x) ((x)) // Normal (right)
#define STEPPER_CORRECT_DIR(x) (-(x)) // Reversed (left)

#define STEPPER_PULSE_PER_REV 800 // pulse/rev
#define RAIL_MM_PER_REV 75 // mm/rev
#define RAIL_MAX_LENGTH_MM 1200
#define STEPPER_MAX_PULSE ((float)RAIL_MAX_LENGTH_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV)

// Define some steppers and the pins the will use
AccelStepper stepper(AccelStepper::DRIVER, 20, 19);

volatile bool reach_zero = false;

uint32_t last_action_time;

#define TIMER_TIMEOUT_US 40

esp_timer_handle_t timer;

void timer_callback(void *arg) {
  if (!digitalRead(14) && !reach_zero) {
    // Serial.println("limit reach");
    stepper.setCurrentPosition(0);
    reach_zero = true;
    stepper.moveTo(STEPPER_CORRECT_DIR(10));
  }
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();
  stepper.run();

  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));
}

// Max
// #define MAX_SPEED_MM 500 // mm/s
// #define ACCELERATION_MM 750 // mm/s/s

// slower for better stability
#define MAX_SPEED_MM 300 // mm/s
#define ACCELERATION_MM 100 // mm/s/s

void setup() {
  Serial.begin(921600);

  pinMode(14, INPUT_PULLUP);

  stepper.setMinPulseWidth(2); // 防止丢步 驱动器要求最小 2us 持续时间
  stepper.setMaxSpeed((float)MAX_SPEED_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV); // 空载最大 750rpm
  stepper.setAcceleration((float)ACCELERATION_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV);

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));

  stepper.setMaxSpeed((float)100 / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV);
  stepper.moveTo(STEPPER_CORRECT_DIR((int)(STEPPER_MAX_PULSE/2)));
  while (!reach_zero) delay(1);
  Serial.println("reach zero");
  stepper.setMaxSpeed((float)MAX_SPEED_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV);

  comm_init();

  last_action_time = millis();
}

void loop() {
  comm_type_t type;
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      ;
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint32_t pos[4];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 4; ++i) pos[i] = ntohl(pos[i]);

      if (pos[0] > STEPPER_MAX_PULSE) {
        Serial.println("out of limit");
      } else {
        stepper.moveTo(STEPPER_CORRECT_DIR(pos[0]));
      }
      pos[0] = STEPPER_CORRECT_DIR(stepper.currentPosition());

      for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
      comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
    } else {
      Serial.println("Wrong Cmd");
    }
  }
  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;

    // Serial.println("No ctrl");

    uint32_t pos[4];
    pos[0] = STEPPER_CORRECT_DIR(stepper.currentPosition());

    for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
    comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
  }

  delay(1);
}
