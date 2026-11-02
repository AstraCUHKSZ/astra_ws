#include <Arduino.h>
#include <SCServo.h>
#include "config.h"
#include "dualMotor.h"
#include "trapTraj.hpp"
#include <esp_task_wdt.h>

SMS_STS sts;

// Rebuild with false to compare against periodic wrist position writes.
constexpr bool kDeduplicateWristCommands = true;
struct WristCommandCache {
  bool valid = false;
  uint16_t position = 0;
  uint16_t speed = 0;
  uint8_t acceleration = 0;
};
static WristCommandCache wrist_commands[3]; // IDs 12, 13, 14 only

static void invalidateWristCommands() {
  for (auto &command : wrist_commands) command.valid = false;
}

#define TIMER_TIMEOUT_US 15000 // change: current_meas_period
// velocity update in servo seems at 50Hz.

esp_timer_handle_t timer;

TrapezoidalTrajectory traj[JOINT_NUM];
float last_pos[JOINT_NUM + NONE_JOINT_NUM] = { 2048, 2048, 2048, 2048, 2048, 2048,  };
float last_vel[JOINT_NUM + NONE_JOINT_NUM] = { 0 };

uint16_t none_joint_goal_pos[NONE_JOINT_NUM] = { 2048, 2048, 2048, 2048,  };
uint16_t raw_goal_pos[JOINT_NUM] = { 2048, 2048,  };

volatile bool protect_triggered = false;
bool torque_enabled = false;
// read_pos() is also used once during boot, before the servo bus is fully
// initialized. Do not latch a communication fault from that startup probe.
static bool communication_protection_armed = false;

#define ACC 100
#define MAX_VEL 10000

#define OVERLOAD_DURATION_CYCLES 10

static uint8_t cnt_load[4 * JOINT_NUM + NONE_JOINT_NUM + 1] = {0};
static uint8_t cnt_temper[4 * JOINT_NUM + NONE_JOINT_NUM + 1] = {0};
static uint8_t cnt_current[4 * JOINT_NUM + NONE_JOINT_NUM + 1] = {0};

void checkAndProtect(int id, int val, int limit, uint8_t* counter, const char* label) {
  if (limit <= 0) return;
  if (std::abs(val) > limit) {
    if (*counter < 255) (*counter)++;
    if (*counter >= OVERLOAD_DURATION_CYCLES) {
      Serial.printf("OVERLOAD: %s limit exceeded on servo #%d: %d > %d. Disabling torque.\n", label, id, std::abs(val), limit);
      protect_triggered = true;
      doSetupTorque(0);
    }
  } else {
    *counter = 0;
  }
}

bool checkCommunicationAndProtect(int id) {
  if (sts.Err != 1) return false;

  Serial.printf("Error reading #%d, check your wire connection\n", id);
  if (communication_protection_armed && torque_enabled && !protect_triggered) {
    Serial.printf("COMMUNICATION ERROR on servo #%d. Disabling torque.\n", id);
    protect_triggered = true;
    doSetupTorque(0);
  }
  return true;
}

void read_pos() {
  int raw_pos[4 * JOINT_NUM];
  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    int id = 4 + i;
    sts.FeedBack(id);
    // Stop this sample here: ReadPos/Load/etc. would otherwise use stale or
    // invalid feedback after a failed transaction.
    if (checkCommunicationAndProtect(id)) return;
    raw_pos[i] = sts.ReadPos(-1);
    
    // Safety checks
    int idx = i;
    checkAndProtect(id, sts.ReadLoad(-1), config.max_load, &cnt_load[idx], "Load");
    checkAndProtect(id, sts.ReadTemper(-1), config.max_temper, &cnt_temper[idx], "Temperature");
    checkAndProtect(id, sts.ReadCurrent(-1), config.max_current, &cnt_current[idx], "Current");
  }
  int raw_pos_non_joint[NONE_JOINT_NUM];
  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    int id = 4 + 4 * JOINT_NUM + i;
    sts.FeedBack(id);
    if (checkCommunicationAndProtect(id)) return;
    raw_pos_non_joint[i] = sts.ReadPos(-1);
    
    // Safety checks
    int idx = 4 * JOINT_NUM + i;
    checkAndProtect(id, sts.ReadLoad(-1), config.max_load, &cnt_load[idx], "Load");
    checkAndProtect(id, sts.ReadTemper(-1), config.max_temper, &cnt_temper[idx], "Temperature");
    checkAndProtect(id, sts.ReadCurrent(-1), config.max_current, &cnt_current[idx], "Current");
  }

  // Zero offest
  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    raw_pos[i] -= config.init_pos[i];
    if (raw_pos[i] < 0) raw_pos[i] += 4096;
    // Move middle from 0 to 2048
    raw_pos[i] += 2048;
    if (raw_pos[i] >= 4096) raw_pos[i] -= 4096;
  }

  // average load into 4 servo
  float pos[JOINT_NUM + NONE_JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    pos[i] = (raw_pos[i * 4] + (4096 - raw_pos[i * 4 + 1]) + raw_pos[i * 4 + 2] + (4096 - raw_pos[i * 4 + 3])) / 4.0;
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    pos[JOINT_NUM + i] = raw_pos_non_joint[i];
  }

  float vel[JOINT_NUM + NONE_JOINT_NUM];
  for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
    float delta = pos[i] - last_pos[i];
    // Wrap-around correction: a single control tick cannot move more than half
    // a revolution, so any |delta| > 2048 must be a 0/4096 boundary crossing.
    if (delta >  2048.0f) delta -= 4096.0f;
    if (delta < -2048.0f) delta += 4096.0f;
    vel[i] = delta / TIMER_TIMEOUT_US * 1000000;
    float filter_vel = 0.2;
    vel[i] = vel[i] * filter_vel + last_vel[i] * (1 - filter_vel);
    // float filter_pos = 0.15860013531; // 1 / (1 + 1 / (2 * pi * f * TIMER_TIMEOUT_S))
    // pos[i] = pos[i] * filter_pos + last_pos[i] * (1 - filter_pos);
    last_pos[i] = pos[i];
    last_vel[i] = vel[i];
  }
}

void doInitJoint(int id, int offset) {
  Serial.printf("INIT_JOINT id=%d offset=%d\n", id, offset);

  sts.EnableTorque(id, 128);
  Serial.printf("Set current position as mid\n");

  sts.unLockEprom(id);//unlock EPROM-SAFE
  int offset_servo = sts.readWord(id, SMS_STS_OFS_L);
  if (offset_servo > 2048) offset_servo = -(offset_servo - 2048);
  Serial.printf("Offset in servo: %d\n", offset_servo);

  offset_servo -= offset;

  if (offset_servo < -2048) offset_servo += 4096;
  if (offset_servo >= 2048) offset_servo -= 4096;

  if (offset_servo < 0) offset_servo = -(offset_servo) + 2048;
  sts.writeWord(id, SMS_STS_OFS_L, offset_servo);
  sts.LockEprom(id);//EPROM-SAFE locked

  offset_servo = sts.readWord(id, SMS_STS_OFS_L);
  if (offset_servo > 2048) offset_servo = -(offset_servo - 2048);
  Serial.printf("Updated offset in servo: %d\n", offset_servo);
}

int setupTorque_enable = 0;
bool updateSetupTorque = false;

void doSetupTorque(int enable) {
  invalidateWristCommands();
  if (enable == 0) {
    torque_enabled = false;
    enable = 0;
  } else if (enable == 128) {} else if (enable == 1 && !protect_triggered) {
    torque_enabled = true;
  } else {
    // Serial.println("Overload protection triggered, cannot enable torque. Reset the controller to clear.");
    torque_enabled = false;
    return;
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    sts.EnableTorque(4 + 4 * JOINT_NUM + i, enable);
  }

  if (enable == 0) {
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.EnableTorque(4 + i, enable);
    }
  } else if (enable == 1) {
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.EnableTorque(4 + i, enable);
    }

    for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
      last_vel[i] = 0;
    }
    read_pos();

    // set current pos as target
    uint16_t pos[JOINT_NUM + NONE_JOINT_NUM];
    for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
      pos[i] = last_pos[i];
    }
    for (int i = 0; i < JOINT_NUM; ++i) {
      traj[i].trajectory_done_ = true;
      traj[i].Xf_ = pos[i];
      traj[i].pos_setpoint_ = pos[i];
    }
    dualMotorUpdatePos(pos);
    Serial.println("setup torque");
  } else if (enable == 128) {
    // MARK: Set Middle Position to 4-11
    // for (int i = 4; i <= 11; ++i) {
    //     // 1. UnLock
    //     sts.unLockEprom(i); 
    //     delay(10); // 等待解锁生效

    //     // 2. Set Max Angle (EPROM Write 1)
    //     sts.writeWord(i, 11, 4095);
    //     delay(50); // !!! 重要：等待 EPROM 写入完成

    //     // 3. Set Mode (EPROM Write 2)
    //     sts.writeByte(i, 33, 0); // 0x21 is 33
    //     delay(50); // !!! 重要：等待 EPROM 写入完成

    //     // 4. Calibration (Trigger EPROM Write 3)
    //     sts.CalibrationOfs(i);
    //     delay(1000); // 等待校准写入完成。这个可能需要久一点，或者为了保险

    //     // 5. Lock
    //     sts.LockEprom(i); 
    //     delay(10);
        
    //     Serial.printf("Set servo #%d current position as mid\n", i);
    // }
    // MARK: Calibrate Gripper (ID 15)
    // -------------------------------------------------------------
    // Force clear offset to read raw physical
    int id = 15;

    sts.unLockEprom(id);
    sts.writeWord(id, SMS_STS_OFS_L, 0); // Set
    sts.LockEprom(id);
    
    unsigned long start_time = millis();
    while (millis() - start_time < 50) {
        // Keep checking to consume time
        sts.FeedBack(id); 
    }
    
    sts.FeedBack(id);
    int current_physical_pos = sts.ReadPos(-1);
    
    // -------------------------------------------------------------
    // Calculate New Offset
    // Target: 3496 (Closed)
    // Formula from empirical test: ScaledPos = Physical - Offset
    // Therefore: Offset = Physical - Target
    //
    // Left Arm Data: Physical=2874, Target=3496.
    // Previous Code (Target-Phys) = 622. Result=2252 (2874-622).
    // We want Result=3496. 
    // Need Offset = 2874 - 3496 = -622.
    // -622 encoded is 2048+622 = 2670.
    // Check: 2874 - (-622) = 3496. Correct.
    // -------------------------------------------------------------
    long new_offset_val = current_physical_pos - 3496;
    
    // -------------------------------------------------------------
    // FORMAT OFFSET for STS SERVO (Sign + Magnitude logic)
    // Positive: 0..2047
    // Negative: 2048..4095 (where 2048+X = -X)
    // -------------------------------------------------------------
    int offset_register_val = 0;
    
    // Normalize to range [-2047, 2047] just in case
    while (new_offset_val > 2047) new_offset_val -= 4096;
    while (new_offset_val < -2047) new_offset_val += 4096;

    offset_register_val = (new_offset_val < 0) ? (2048 + (-new_offset_val)) : new_offset_val;

    Serial.printf("Calibrating Gripper Servo ID %d\n", id);
    Serial.printf(" Current Physical Pos: %d\n", current_physical_pos);
    Serial.printf(" Calculated New Offset: %ld\n", new_offset_val);
    Serial.printf(" Offset Register Value to Write: %d\n", offset_register_val);

    sts.unLockEprom(id);
    sts.writeWord(id, SMS_STS_OFS_L, offset_register_val); 
    sts.LockEprom(id);
    int verify = sts.readWord(id, SMS_STS_OFS_L); // Verify write
    Serial.printf(" Verified Offset Register Value: %d\n", verify);
    int verifyPos = sts.ReadPos(id);
    Serial.printf(" Verified Position after Offset Applied: %d\n", verifyPos);

    // Save gripper offset to flash config so it can be restored on next boot
    config.gripper_offset_inited = true;
    config.gripper_offset = offset_register_val;
    Serial.printf(" Saved gripper offset to config: %d\n", config.gripper_offset);

    // doInitJoint(id, new_offset_val);
    // -------------------------------------------------------------

    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.unLockEprom(4 + i); //unlock EPROM-SAFE
      // sts.PWMMode(4 + i)
      sts.writeByte(4 + i, 0x21, 2); // set mode to PWM
      sts.LockEprom(4 + i); //EPROM-SAFE locked
    }

    int init_pos0[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos0[i] = sts.ReadPos(4 + i);
      // if (!(20 < init_pos0[i] && init_pos0[i] < 4096 - 20)) {
      //   Serial.print(init_pos0[i]);
      //   Serial.print("Maybe cause wrong init_pos0, check power status, or reinstall the servo #"); Serial.print(4+i); Serial.println();
      //   while (1) ;
      // }
    }

    // preload
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, config.joint_backlash_compensate_feedforward);
    }

    // wait for stable
    sleep(1);

    int init_pos1[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos1[i] = sts.ReadPos(4 + i);
    }

    // inversed preload
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, config.joint_backlash_compensate_feedforward + 1024); // negative as scs goal time format
    }

    // wait for stable
    sleep(1);

    int init_pos2[4 * JOINT_NUM];
    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      init_pos2[i] = sts.ReadPos(4 + i);
    }

    for (int i = 0; i < 4 * JOINT_NUM; ++i) {
      int pos1 = init_pos1[i];
      int pos2 = init_pos2[i];
      int diff = std::abs(pos1 - pos2);
      if (diff > 2048) {  // 超过半圈，说明跨越了 0/4096 边界
        diff = 4096 - diff;
        // 如果两点跨越边界，把较小的值加到 4096 以上，方便后续计算
        if (pos1 < pos2) {
          pos1 += 4096;
        } else {
          pos2 += 4096;
        }
        Serial.print("Boundary crossed. Adjusted positions: "); Serial.print(pos1); Serial.print(","); Serial.print(pos2); Serial.print(", Diff: "); Serial.println(diff);
      }
      if (diff > 200) {
        Serial.print(init_pos1[i]); Serial.print(","); Serial.print(init_pos2[i]);
        Serial.print(",");
        Serial.print(std::abs(init_pos1[i] - init_pos2[i]));
        Serial.print("Gap is too wide, check power status, or reinstall the servo #"); Serial.print(4+i); Serial.println();
        while (1) ;
      }

      config.init_pos_inited = true;

      int mid = (pos1 + pos2) / 2;
      if (mid >= 4096) {
          mid -= 4096;
      }

      config.init_pos[i] = mid;
    }
    write_config();
  }
}

void setupTorque(int enable) {
  setupTorque_enable = enable;
  updateSetupTorque = true;
}

volatile bool debug_mode = false;

void debugSetServoPWM(int id, int pwm) {
  if (debug_mode) {
    sts.writeWord(id, SCSCL_GOAL_TIME_L, pwm);
  }
}

void debugSetServoPosition(int id, int position) {
  if (debug_mode) {
    sts.WritePosEx(id, position, config.non_joint_vel_max, config.non_joint_acc);
  }
}

void debugGetServoStatus(int id, int16_t* pos, int16_t* speed, int16_t* load, uint8_t* voltage, uint8_t* temper) {
    if (sts.FeedBack(id) != -1) {
        *pos = sts.ReadPos(-1);
        *speed = sts.ReadSpeed(-1);
        *load = sts.ReadLoad(-1);
        *voltage = sts.ReadVoltage(-1);
        *temper = sts.ReadTemper(-1);
    } else {
        *pos = 0;
        *speed = 0;
        *load = 0;
        *voltage = 0;
        *temper = 0;
    }
}

float pidtune_kp = 0, pidtune_kd = 0, pidtune_ki = 0, pidtune_ki_max = 800, pidtune_kp2 = 0, pidtune_kp2_err_point = 0, pidtune_ki_clip_thres = 10, pidtune_ki_clip_coef = 0.5;

void timer_callback(void *arg) {
  if (updateSetupTorque) {
    updateSetupTorque = false;
    doSetupTorque(setupTorque_enable);
  }

  if (debug_mode) {
    invalidateWristCommands();
    return;
  }

  if (!torque_enabled) {
    return;
  }

  if (!config.init_pos_inited) {
    Serial.println("Joint pos is not inited");
    return;
  }

  read_pos();

  if (!torque_enabled) {
    return;
  }

  float goal_pos[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    traj[i].update();
    goal_pos[i] = traj[i].pos_setpoint_;
  }

  // static float last_goal_pos[JOINT_NUM];
  // static float last_goal_pos_inited = false;
  // if (!last_goal_pos_inited) {
  //   for (int i = 0; i < JOINT_NUM; ++i) {
  //     last_goal_pos[i] = goal_pos[i];
  //   }
  //   last_goal_pos_inited = true;
  // }

  float kp = pidtune_kp, kd = pidtune_kd, ki = pidtune_ki, kp2 = pidtune_kp2, kp2_err_point = pidtune_kp2_err_point;

  static float last_err[JOINT_NUM];
  static float i_out[JOINT_NUM] = {};
  float out[JOINT_NUM] = {};
  float debug_signal[JOINT_NUM];

  // dithering to overcome coulomb friction
  // Ref: https://ieeexplore.ieee.org/document/7139742
  static float sticktion_compensation = 60;
  sticktion_compensation = -sticktion_compensation;

  for (int i = 0; i < JOINT_NUM; ++i) {
    float err = goal_pos[i] - last_pos[i];
    if (err > 2048.0f) err -= 4096.0f;
    if (err < -2048.0f) err += 4096.0f;

    // float goal_vel = goal_pos[i] - last_goal_pos[i];
    // last_goal_pos[i] = goal_pos[i];

    // float friction_compensation = goal_vel * 0.30385482 + 61.1004804 * (goal_vel > 0 ? 1 : -1);

    float p_out = kp * err + (kp2 - kp) * (err > kp2_err_point ? err - kp2_err_point : 0) + (kp2 - kp) * (err < -kp2_err_point ? err + kp2_err_point : 0);
    i_out[i] += ki * err;
    if (i_out[i] > pidtune_ki_max) i_out[i] = pidtune_ki_max;
    if (i_out[i] < -pidtune_ki_max) i_out[i] = -pidtune_ki_max;
    // debug_signal[i] = 200;
    // if (std::abs(last_vel[i]) > pidtune_ki_clip_thres) {
    //   i_out[i] = i_out[i] * pidtune_ki_clip_coef;
    //   debug_signal[i] = 400;
    // }
    float d_out = kd * (err - last_err[i]);

    out[i] = /*friction_compensation +*/ sticktion_compensation + p_out + i_out[i] + d_out;

    // Serial.print(p_out);
    // Serial.print(",");
    // Serial.print(i_out[i]);
    // Serial.print(",");
    // Serial.print(d_out);
    // Serial.print(",");
    // Serial.print(out[i]);
    // Serial.print(",");
    // Serial.print("  ");

    last_err[i] = err;
    delay(1);
  }
  // Serial.println(millis());

  float raw_out[4 * JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i) {
    // float backlash_compensate_feedforward = config.joint_backlash_compensate_feedforward;
    float backlash_compensate_feedforward = 150;
    raw_out[i * 4] = backlash_compensate_feedforward + out[i];
    raw_out[i * 4 + 1] = backlash_compensate_feedforward + -out[i];
    raw_out[i * 4 + 2] = backlash_compensate_feedforward + out[i];
    raw_out[i * 4 + 3] = backlash_compensate_feedforward + -out[i];
  }

  for (int i = 0; i < 4 * JOINT_NUM; ++i) {
    int ready_send = -raw_out[i]; // Max: 1000 as servo document
    if (ready_send > 800) ready_send = 800;
    if (ready_send < -800) ready_send = -800;
    if (ready_send < 0) ready_send = -(ready_send) + 1024;
    sts.writeWord(4 + i, SCSCL_GOAL_TIME_L, ready_send);
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    const int id = 4 + 4 * JOINT_NUM + i;
    const uint16_t position = none_joint_goal_pos[i];
    const uint16_t speed = config.non_joint_vel_max;
    const uint8_t acceleration = config.non_joint_acc;
    if (kDeduplicateWristCommands && id >= 12 && id <= 14) {
      auto &cached = wrist_commands[id - 12];
      if (cached.valid && cached.position == position &&
          cached.speed == speed && cached.acceleration == acceleration) continue;
      // Invalidate before attempting a changed command; a failed write must
      // also be retried if the next target returns to the earlier cached value.
      cached.valid = false;
      const int result = sts.WritePosEx(id, position, speed, acceleration);
      if (result == 1 && sts.Level != 0 && sts.Error == 0) {
        cached.position = position;
        cached.speed = speed;
        cached.acceleration = acceleration;
        cached.valid = true;
      }
    } else {
      sts.WritePosEx(id, position, speed, acceleration);
    }
  }

  // In timer thread, Racing condition with main thread

  // Serial.println();

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(raw_goal_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(goal_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_pos[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(debug_signal[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(last_vel[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < JOINT_NUM; ++i) {
  //   Serial.print(out[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // for (int i = 0; i < 4 * JOINT_NUM; ++i) {
  //   Serial.print(raw_out[i]);
  //   Serial.print(",");
  // }
  // Serial.print("  ");

  // Serial.println(millis());
  // print time for debug
}

void dualMotorSetup() {
#define S_RXD 18
#define S_TXD 19
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  while (!Serial1) delay(1);
  sts.pSerial = &Serial1;

  // Restore saved gripper offset from flash config
  if (config.gripper_offset_inited) {
    sts.unLockEprom(15);
    sts.writeWord(15, SMS_STS_OFS_L, config.gripper_offset);
    sts.LockEprom(15);
    Serial.printf("Restored gripper offset from config: %d\n", config.gripper_offset);
  } else {
    Serial.println("Gripper offset not calibrated yet, run torque=128 to calibrate.");
  }

  sts.writeWord(15, SMS_STS_TORQUE_LIMIT_L, 500); // Protect servo

  read_pos();
  communication_protection_armed = true;

  for (int i = 0; i < JOINT_NUM; ++i) {
    // traj[i].config_.vel_limit = config.joint_vel_max;
    // traj[i].config_.accel_limit = config.joint_acc;
    // traj[i].config_.decel_limit = config.joint_acc;
  }

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, TIMER_TIMEOUT_US));
}

void dualMotorUpdatePos(uint16_t pos[]) {
  if (!torque_enabled) {
    setupTorque(1);
  }
  for (int i = 0; i < JOINT_NUM; ++i) {
    raw_goal_pos[i] = pos[i];
    traj[i].planTrapezoidal(pos[i], traj[i].pos_setpoint_, traj[i].vel_setpoint_);
  }

  for (int i = 0; i < NONE_JOINT_NUM; ++i) {
    none_joint_goal_pos[i] = pos[JOINT_NUM + i];
  }
}

void dualMotorReadPos(uint16_t read_pos[]) {
  for (int i = 0; i < JOINT_NUM + NONE_JOINT_NUM; ++i) {
    read_pos[i] = last_pos[i];
  }
}
