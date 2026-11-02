#include <Arduino.h>
#include <SCServo.h>
#include <lwip/sockets.h>
#include <Adafruit_SSD1306.h>
#include "EIShaper.h"
#include "comm.h"
#include "main.h"
#include "dualMotor.h"
#include "config.h"

extern SMS_STS sts;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void InitScreen() {
// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21
  Wire.begin(S_SDA, S_SCL);
// SSD1306: 0x3C
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

int ctrl_cnt;
int feedback_cnt;
uint32_t last_stat_time;

uint32_t last_action_time;

uint32_t last_ping_time;
uint32_t last_pong_time;

// 带有检查功能的设置函数
void setServoModeIfChanged(int id, int targetMode) {
    // 尝试读取当前模式 (寄存器 33)
    // 注意：readByte 如果返回 -1 表示通信失败
    int currentMode = sts.readByte(id, SMS_STS_MODE);
    Serial.printf("ID:%d CurrentMode:%d TargetMode:%d\n", id, currentMode, targetMode);
    
    // if (currentMode != targetMode && currentMode != -1) {
    //     Serial.printf("Changing ID:%d to Mode:%d\n", id, targetMode);
    //     sts.unLockEprom(id);
    //     sts.writeByte(id, SMS_STS_MODE, targetMode);
    //     sts.LockEprom(id);
    //     delay(10); // 短暂延迟确保写入稳定
    // } else if (currentMode == -1) {
    //     Serial.printf("Warning: ID:%d not found!\n", id);
    // }
}

void initServoModes() {
    int servoIDs[] = {4, 5, 6, 7, 8, 9, 10, 11};// , 12, 13, 14, 15

    // 初始化普通舵机模式舵机
    for (int id : servoIDs) {
      setServoModeIfChanged(id, 2);
    }

    setServoModeIfChanged(15, 0); // Gripper servo to position mode
}

void setup() {
  comm_init();

  InitScreen();

  configInit();

  dualMotorSetup();

  setupTorque(0);

  uint16_t pos[8];
  dualMotorReadPos(pos);
  if (config.EIShaper_enabled) {
    EIShaperInit(pos);
  }

  ctrl_cnt = 0;
  feedback_cnt = 0;
  last_stat_time = millis();

  last_action_time = millis();

  last_ping_time = 0;
  last_pong_time = 0;

  initServoModes();
}

#define FLOAT_TO_U32(x) (*(uint32_t *)&(x))
#define U32_TO_FLOAT(x) (*(float *)&(x))
#define U32_TO_I32(x) (*(int *)&(x))

void loop() {
  comm_type_t type;
  uint8_t buf[COMM_PAYLOAD_SIZE_MAX];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    // TODO PING PONG
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      last_pong_time = millis();
    } else if (type == COMM_TYPE_TORQUE) {
      setupTorque(buf[0]);
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint16_t pos[8];
      memcpy(pos, buf, sizeof pos);
      for (int i = 0; i < 6; ++i) pos[i] = ntohs(pos[i]);

      // EIShaper
      if (config.EIShaper_enabled) {
        EIShaperApply(pos);
      }

      ++ctrl_cnt;
      dualMotorUpdatePos(pos);

      ++feedback_cnt;
      dualMotorReadPos(pos);

      for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
      comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
    } else if (type == COMM_TYPE_CONFIG_WRITE) {
      uint32_t cmd[2];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 2; ++i) cmd[i] = ntohl(cmd[i]);

      if (cmd[0] == 0x01) {
        config.EIShaper_enabled = (bool)cmd[1];
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = (uint32_t)config.EIShaper_enabled;
      } else if (cmd[0] == 0x02) {
        config.EIShaper_freq = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_freq);
      } else if (cmd[0] == 0x03) {
        config.EIShaper_V = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_V);
      } else if (cmd[0] == 0x04) {
        config.EIShaper_ctrl_freq = U32_TO_FLOAT(cmd[1]);
        write_config();
        uint16_t pos[8];
        dualMotorReadPos(pos);
        if (config.EIShaper_enabled) {
          EIShaperInit(pos);
        }
        cmd[1] = FLOAT_TO_U32(config.EIShaper_ctrl_freq);;
      } else if (cmd[0] == 0x05) {
        config.joint_vel_max = U32_TO_FLOAT(cmd[1]);
        write_config();
        cmd[1] = FLOAT_TO_U32(config.joint_vel_max);
      } else if (cmd[0] == 0x06) {
        config.joint_acc = U32_TO_FLOAT(cmd[1]);
        write_config();
        cmd[1] = FLOAT_TO_U32(config.joint_acc);
      } else if (cmd[0] == 0x07) {
        config.joint_backlash_compensate_feedforward = (cmd[1]);
        write_config();
        cmd[1] = (config.joint_backlash_compensate_feedforward);
      } else if (cmd[0] == 0x08) {
        config.non_joint_vel_max = (cmd[1]);
        write_config();
        cmd[1] = (config.non_joint_vel_max);
      } else if (cmd[0] == 0x09) {
        config.non_joint_acc = (cmd[1]);
        write_config();
        cmd[1] = (config.non_joint_acc);
      } else if (cmd[0] == 0x0A) {
        config.max_load = (cmd[1]);
        write_config();
        cmd[1] = (config.max_load);
      } else if (cmd[0] == 0x0B) {
        config.max_temper = (cmd[1]);
        write_config();
        cmd[1] = (config.max_temper);
      } else if (cmd[0] == 0x0C) {
        config.max_current = (cmd[1]);
        write_config();
        cmd[1] = (config.max_current);
      } else {
        Serial.println("Unknown command");
      }

      for (int i = 0; i < 2; ++i) cmd[i] = htonl(cmd[i]);
      memcpy(buf, cmd, sizeof cmd);
      comm_send_blocking(COMM_TYPE_CONFIG_FEEDBACK, (uint8_t *)buf);
    } else if (type == COMM_TYPE_CONFIG_READ) {
      uint32_t cmd[2];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 2; ++i) cmd[i] = ntohl(cmd[i]);

      if (cmd[0] == 0x01) {
        cmd[1] = (uint32_t)config.EIShaper_enabled;
      } else if (cmd[0] == 0x02) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_freq);
      } else if (cmd[0] == 0x03) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_V);
      } else if (cmd[0] == 0x04) {
        cmd[1] = FLOAT_TO_U32(config.EIShaper_ctrl_freq);
      } else if (cmd[0] == 0x05) {
        cmd[1] = FLOAT_TO_U32(config.joint_vel_max);
      } else if (cmd[0] == 0x06) {
        cmd[1] = FLOAT_TO_U32(config.joint_acc);
      } else if (cmd[0] == 0x07) {
        cmd[1] = (config.joint_backlash_compensate_feedforward);
      } else if (cmd[0] == 0x08) {
        cmd[1] = (config.non_joint_vel_max);
      } else if (cmd[0] == 0x09) {
        cmd[1] = (config.non_joint_acc);
      } else if (cmd[0] == 0x0A) {
        cmd[1] = (config.max_load);
      } else if (cmd[0] == 0x0B) {
        cmd[1] = (config.max_temper);
      } else if (cmd[0] == 0x0C) {
        cmd[1] = (config.max_current);
      } else {
        Serial.println("Unknown command");
      }

      for (int i = 0; i < 2; ++i) cmd[i] = htonl(cmd[i]);
      memcpy(buf, cmd, sizeof cmd);
      comm_send_blocking(COMM_TYPE_CONFIG_FEEDBACK, (uint8_t *)buf);
    } else if (type == COMM_TYPE_PIDTUNE) {
      uint32_t cmd[8];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 8; ++i) cmd[i] = ntohl(cmd[i]);
      pidtune_kp = U32_TO_FLOAT(cmd[0]);
      pidtune_ki = U32_TO_FLOAT(cmd[1]);
      pidtune_kd = U32_TO_FLOAT(cmd[2]);
      pidtune_ki_max = U32_TO_FLOAT(cmd[5]);
      pidtune_kp2 = U32_TO_FLOAT(cmd[6]);
      pidtune_kp2_err_point = U32_TO_FLOAT(cmd[7]);
      pidtune_ki_clip_thres = U32_TO_FLOAT(cmd[3]);
      pidtune_ki_clip_coef = U32_TO_FLOAT(cmd[4]);
      Serial.printf("PIDTUNE kp=%.2f ki=%.2f kd=%.2f ki_max=%.2f kp2=%.2f kp2_err_point=%.2f ki_clip_thres=%.2f ki_clip_coef=%.2f\n", pidtune_kp, pidtune_ki, pidtune_kd, pidtune_ki_max, pidtune_kp2, pidtune_kp2_err_point, pidtune_ki_clip_thres, pidtune_ki_clip_coef);
    } else if (type == COMM_TYPE_DEBUG) {
      uint32_t cmd[8];
      memcpy(cmd, buf, sizeof cmd);
      for (int i = 0; i < 8; ++i) cmd[i] = ntohl(cmd[i]);
      
      uint32_t subcmd = cmd[0];
      if (subcmd == 0) { // Set Debug Mode
        debug_mode = (cmd[1] != 0);
        if (debug_mode) {
           Serial.println("DEBUG MODE ENABLED");
        } else {
           Serial.println("DEBUG MODE DISABLED");
        }
      } else if (subcmd == 1) { // Set PWM
        int id = cmd[1];
        int pwm = (int)cmd[2];
        debugSetServoPWM(id, pwm);
      } else if (subcmd == 2) { // Get Info
        int id = cmd[1];
        int16_t pos, speed, load;
        uint8_t voltage, temper;
        debugGetServoStatus(id, &pos, &speed, &load, &voltage, &temper);
        
        cmd[0] = 2; // Echo subcmd
        cmd[1] = id;
        cmd[2] = (uint32_t)pos;
        cmd[3] = (uint32_t)speed;
        cmd[4] = (uint32_t)load;
        cmd[5] = (uint32_t)voltage;
        cmd[6] = (uint32_t)temper;
        cmd[7] = 0; // padding

        for (int i = 0; i < 8; ++i) cmd[i] = htonl(cmd[i]);
        comm_send_blocking(COMM_TYPE_DEBUG, (uint8_t *)cmd);
      } else if (subcmd == 3) { // Set Torque Enable
        int id = cmd[1];
        int enable = cmd[2];
        sts.EnableTorque(id, enable);
      } else if (subcmd == 4) { // Set Position
        int id = cmd[1];
        int position = cmd[2];
        debugSetServoPosition(id, position);
      } else if (subcmd == 5) { // Read servo EEPROM register
        const int id = cmd[1];
        const int address = cmd[2];
        const int width = cmd[3];
        uint32_t value = 0;
        uint32_t status = 0;

        // EEPROM occupies addresses 0..39 on the SMS/STS control table.
        // Requiring debug mode prevents this transaction from racing the
        // periodic servo-bus control loop.
        if (!debug_mode) {
          status = 1; // Debug mode required.
        } else if (id < 0 || id > 253) {
          status = 2; // Invalid servo ID.
        } else if ((width != 1 && width != 2) || address < 0 ||
                   address + width > 40) {
          status = 3; // Invalid EEPROM address/width.
        } else {
          const int read_value = width == 1
              ? sts.readByte(id, address)
              : sts.readWord(id, address);
          if (read_value < 0) {
            status = 4; // Servo-bus read failed.
          } else {
            value = (uint32_t)read_value;
          }
        }

        cmd[0] = 5;
        cmd[1] = id;
        cmd[2] = address;
        cmd[3] = width;
        cmd[4] = value;
        cmd[5] = status;
        cmd[6] = 0;
        cmd[7] = 0;
        for (int i = 0; i < 8; ++i) cmd[i] = htonl(cmd[i]);
        comm_send_blocking(COMM_TYPE_DEBUG, (uint8_t *)cmd);
      } else if (subcmd == 6) { // Write and verify servo EEPROM register
        const int original_id = cmd[1];
        const int address = cmd[2];
        const int width = cmd[3];
        const uint32_t requested_value = cmd[4];
        int resulting_id = original_id;
        uint32_t readback = 0;
        uint32_t status = 0;

        if (!debug_mode) {
          status = 1; // Debug mode required.
        } else if (original_id < 0 || original_id > 253) {
          status = 2; // Invalid servo ID.
        } else if ((width != 1 && width != 2) || address < 0 ||
                   address + width > 40) {
          status = 3; // Invalid EEPROM address/width.
        } else if ((width == 1 && requested_value > 0xff) ||
                   (width == 2 && requested_value > 0xffff)) {
          status = 5; // Value does not fit the requested width.
        } else if (address <= SMS_STS_ID &&
                   address + width > SMS_STS_ID &&
                   (address != SMS_STS_ID || width != 1 || requested_value > 253)) {
          status = 6; // Invalid ID-register write.
        } else {
          sts.unLockEprom(original_id);
          if (width == 1) {
            sts.writeByte(original_id, address, (uint8_t)requested_value);
          } else {
            sts.writeWord(original_id, address, (uint16_t)requested_value);
          }

          // An ID change takes effect immediately, so subsequent lock/read
          // transactions must address the new ID.
          if (address == SMS_STS_ID) resulting_id = (int)requested_value;
          sts.LockEprom(resulting_id);

          const int verified_value = width == 1
              ? sts.readByte(resulting_id, address)
              : sts.readWord(resulting_id, address);
          if (verified_value < 0) {
            status = 4; // Servo-bus readback failed.
          } else {
            readback = (uint32_t)verified_value;
            if (readback != requested_value) status = 7; // Verify mismatch.
          }
        }

        cmd[0] = 6;
        cmd[1] = original_id;
        cmd[2] = address;
        cmd[3] = width;
        cmd[4] = readback;
        cmd[5] = status;
        cmd[6] = resulting_id;
        cmd[7] = 0;
        for (int i = 0; i < 8; ++i) cmd[i] = htonl(cmd[i]);
        comm_send_blocking(COMM_TYPE_DEBUG, (uint8_t *)cmd);
      }
    } else {
      Serial.println("Unknown comm type");
    }
  }

  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;

    // Serial.println("No ctrl");

    uint16_t pos[8];
    ++feedback_cnt;
    dualMotorReadPos(pos);

    for (int i = 0; i < 6; ++i) pos[i] = htons(pos[i]);
    comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);

    // follower 已经10ms没有收到信号了
    ;
  }

  uint32_t this_stat_time = millis();
  if (this_stat_time - last_stat_time > 1000) {
    // Update Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    float ctrl_freq = (float)ctrl_cnt / (this_stat_time - last_stat_time) * 1000;
    float feeback_freq = (float)feedback_cnt / (this_stat_time - last_stat_time) * 1000;
    ctrl_cnt = 0;
    feedback_cnt = 0;
    display.print(millis() / 1000);
    display.print(F("s C:"));
    display.print(ctrl_freq, 1);
    display.println(F("Hz"));

    display.print(F("F:"));
    display.print(feeback_freq, 1);
    display.print(F("Hz W:"));
    display.println(Serial.availableForWrite());

    display.print(F("Act age:"));
    display.print(this_stat_time - last_action_time);
    display.println(F("ms"));

    display.display();

    last_stat_time = this_stat_time;
  }

  delay(1); // 让出线程
}
