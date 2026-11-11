#include <AccelStepper.h>
#include <Arduino.h>
#include <comm.h>
#include <lwip/sockets.h>

#define STEPPER_CORRECT_DIR(x) ((x)) // Normal (right)
// #define STEPPER_CORRECT_DIR(x) (-(x)) // Reversed (left)

#define STEPPER_PULSE_PER_REV 800 // pulse/rev
#define RAIL_MM_PER_REV 75 // mm/rev
#define RAIL_MAX_LENGTH_MM 1200
#define STEPPER_MAX_PULSE ((float)RAIL_MAX_LENGTH_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV)

// Define some steppers and the pins the will use
AccelStepper stepper(AccelStepper::DRIVER, 20, 19);

// 状态：上电后未初始化，不会移动；由上位机 SET_POS 或 HOME(找到光电门) 完成初始化
volatile bool initialized = false;
volatile bool homing = false;

portMUX_TYPE stepper_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t last_action_time;

#define TIMER_TIMEOUT_US 40

esp_timer_handle_t timer;

// 三个 define 在下方，这里先声明
static float normal_speed_pulse();
static float homing_speed_pulse();

void timer_callback(void *arg) {
  portENTER_CRITICAL(&stepper_mux);
  if (homing) {
    if (!digitalRead(14)) {
      // 找到光电门
      stepper.setCurrentPosition(0);
      stepper.moveTo(STEPPER_CORRECT_DIR(10));
      stepper.setMaxSpeed(normal_speed_pulse());
      initialized = true;
      homing = false;
    } else if (stepper.distanceToGo() == 0) {
      // 走完半程仍未找到光电门，放弃，保持原初始化状态
      stepper.setMaxSpeed(normal_speed_pulse());
      homing = false;
    }
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
  portEXIT_CRITICAL(&stepper_mux);

  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));
}

// Max
// #define MAX_SPEED_MM 500 // mm/s
// #define ACCELERATION_MM 750 // mm/s/s

// slower for better stability
#define MAX_SPEED_MM 300 // mm/s
#define ACCELERATION_MM 100 // mm/s/s

static float normal_speed_pulse() {
  return (float)MAX_SPEED_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV;
}

static float homing_speed_pulse() {
  return (float)100 / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV;
}

// 发送反馈：pos[0] 当前位置，pos[1] 状态位 bit0=initialized bit1=homing
static void send_feedback() {
  uint32_t pos[4] = {0, 0, 0, 0};
  portENTER_CRITICAL(&stepper_mux);
  pos[0] = STEPPER_CORRECT_DIR(stepper.currentPosition());
  pos[1] = (initialized ? 1 : 0) | (homing ? 2 : 0);
  portEXIT_CRITICAL(&stepper_mux);
  for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
  comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
}

void setup() {
  Serial.begin(921600);

  pinMode(14, INPUT_PULLUP);

  stepper.setMinPulseWidth(2); // 防止丢步 驱动器要求最小 2us 持续时间
  stepper.setMaxSpeed(normal_speed_pulse()); // 空载最大 750rpm
  stepper.setAcceleration((float)ACCELERATION_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV);

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));

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

      if (!initialized || homing) {
        Serial.println("not ready, ignore ctrl"); // 未初始化或正在回零，不运动
      } else if (pos[0] > STEPPER_MAX_PULSE) {
        Serial.println("out of limit");
      } else {
        portENTER_CRITICAL(&stepper_mux);
        stepper.moveTo(STEPPER_CORRECT_DIR(pos[0]));
        portEXIT_CRITICAL(&stepper_mux);
      }
      send_feedback();
    } else if (type == COMM_TYPE_SET_POS) {
      last_action_time = millis();

      uint32_t pos[4];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 4; ++i) pos[i] = ntohl(pos[i]);

      if (pos[0] > STEPPER_MAX_PULSE) {
        Serial.println("out of limit");
      } else {
        portENTER_CRITICAL(&stepper_mux);
        homing = false;
        stepper.setMaxSpeed(normal_speed_pulse());
        stepper.setCurrentPosition(STEPPER_CORRECT_DIR(pos[0])); // 同时清零速度、目标位置=当前位置
        initialized = true;
        portEXIT_CRITICAL(&stepper_mux);
      }
      send_feedback();
    } else if (type == COMM_TYPE_HOME) {
      last_action_time = millis();

      portENTER_CRITICAL(&stepper_mux);
      stepper.setMaxSpeed(homing_speed_pulse());
      stepper.move(STEPPER_CORRECT_DIR((int)(STEPPER_MAX_PULSE / 2))); // 相对当前位置最多走半程
      homing = true;
      portEXIT_CRITICAL(&stepper_mux);
      send_feedback();
    } else {
      Serial.println("Wrong Cmd");
    }
  }
  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;
    send_feedback();
  }

  delay(1);
}
