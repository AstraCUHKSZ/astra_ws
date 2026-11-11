import asyncio
import json
import logging
import time

import numpy as np
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
import math

from astra_teleop.process import get_solve
from astra_teleop_web.webserver import WebServer

logger = logging.getLogger(__name__)

POSE_FILTER_LOG_PATH = "/tmp/astra_teleop_pose_filter.jsonl"
pose_filter_logger = logging.getLogger("astra_teleop.pose_filter")
if not pose_filter_logger.handlers:
    pose_filter_log_handler = logging.FileHandler(POSE_FILTER_LOG_PATH)
    pose_filter_log_handler.setFormatter(logging.Formatter("%(message)s"))
    pose_filter_logger.addHandler(pose_filter_log_handler)
    pose_filter_logger.setLevel(logging.INFO)
    pose_filter_logger.propagate = False

GRIPPER_MAX = 0.055
INITIAL_LIFT_DISTANCE = 0.916

FAR_SEEING_HEAD_TILT = 0.26

# Keep the low-pass filter active while allowing the stationary pose dead zone
# to be disabled independently for testing or higher-response teleoperation.
POSE_DEADZONE_ENABLED = False

# Independently block left/right handle motion from publishing arm goals.
# reset_arm() uses a separate publication path and is intentionally unaffected.
ARM_MOVEMENT_LOCKED = {
    "left": False,
    "right": False,
}


class PoseMotionFilter:
    MODE_CONFIG = {
        False: {
            "position_cutoff": 1.2,
            "rotation_cutoff": 0.9,
            "stationary_position": 0.004,
            "stationary_rotation": math.radians(1.5),
            "release_position": 0.007,
            "release_rotation": math.radians(2.5),
        },
        True: {
            "position_cutoff": 0.30,
            "rotation_cutoff": 0.30,
            "stationary_position": 0.003,
            "stationary_rotation": math.radians(1.0),
            "release_position": 0.005,
            "release_rotation": math.radians(2.0),
        },
        "more_percise": {
            "position_cutoff": 0.25,
            "rotation_cutoff": 0.25,
            # Motion is scaled down in this mode, so its dead zone must also be
            # smaller or intentional fine motion would be swallowed.
            "stationary_position": 0.002,
            "stationary_rotation": math.radians(1.25),
            "release_position": 0.003,
            "release_rotation": math.radians(1.75),
        },
    }

    def __init__(self, side):
        self.side = side
        self.reset()

    def reset(self):
        self.output_pq = None
        self.last_raw_pq = None
        self.last_time = None
        self.state = "stationary"
        self.release_count = 0
        self.stationary_count = 0
        self.history = []
        self.channel_state = {"position": "stationary", "rotation": "stationary"}
        self.quiet_since = {"position": None, "rotation": None}
        self.release_since = {"position": None, "rotation": None}
        self.anchor = None
        self.last_logged_state = None

    @staticmethod
    def _alpha(cutoff, dt):
        return 1 - math.exp(-2 * math.pi * cutoff * dt)

    def update(self, transform, mode):
        config = self.MODE_CONFIG[mode]
        now = time.monotonic()
        raw = pt.pq_from_transform(transform)
        gap = 0.0 if self.last_time is None else max(0.0, now - self.last_time)
        # A dropout must not count as evidence of quiet motion or sustained release.
        if gap > 0.2:
            self.history.clear()
            self.quiet_since = dict.fromkeys(self.channel_state)
            self.release_since = dict.fromkeys(self.channel_state)
        self.history.append((now, raw.copy()))
        self.history = [(t, p) for t, p in self.history if now - t <= 0.4]
        if self.output_pq is None:
            self.output_pq = raw.copy()
            self.anchor = raw.copy()
        # Use a robust window center for the lock anchor; align quaternion signs
        # before averaging so q/-q representation changes cannot cancel it out.
        samples = np.array([p for _, p in self.history])
        center = raw.copy()
        center[:3] = np.median(samples[:, :3], axis=0)
        qs = samples[:, 3:].copy()
        qs[np.dot(qs, raw[3:]) < 0] *= -1
        center[3:] = np.mean(qs, axis=0)
        center[3:] /= np.linalg.norm(center[3:])
        spread = {
            "position": float(np.percentile(np.linalg.norm(samples[:, :3] - center[:3], axis=1), 80)),
            "rotation": float(np.percentile([pr.quaternion_dist(q, center[3:]) for q in qs], 80)),
        }
        ready = len(self.history) >= 4 and now - self.history[0][0] >= 0.25
        dt = min(gap, 0.08)  # bounded recovery step, even after a long dropout
        alphas = {}
        errors = {}
        for name, sl in (("position", slice(0, 3)), ("rotation", slice(3, 7))):
            def distance(a, b):
                return float(np.linalg.norm(a - b)) if name == "position" else float(pr.quaternion_dist(a, b))
            errors[name] = distance(raw[sl], self.output_pq[sl])
            if not POSE_DEADZONE_ENABLED:
                self.channel_state[name] = "moving"
                self.quiet_since[name] = None
                self.release_since[name] = None
            elif self.channel_state[name] == "stationary":
                if distance(raw[sl], self.anchor[sl]) > config["release_" + name]:
                    if self.release_since[name] is None:
                        self.release_since[name] = now
                    elif now - self.release_since[name] >= 0.08:
                        self.channel_state[name] = "moving"
                        self.quiet_since[name] = None
                        self.release_since[name] = None
                else:
                    self.release_since[name] = None
            alpha = 0.0
            if self.channel_state[name] == "moving":
                alpha = self._alpha(config[name + "_cutoff"], dt)
                if name == "position":
                    self.output_pq[sl] += alpha * (raw[sl] - self.output_pq[sl])
                else:
                    self.output_pq[sl] = pr.quaternion_slerp(self.output_pq[sl], raw[sl], alpha, shortest_path=True)
                if POSE_DEADZONE_ENABLED:
                    quiet = (ready and spread[name] <= config["stationary_" + name]
                             and distance(center[sl], self.output_pq[sl]) <= config["stationary_" + name])
                    if quiet:
                        if self.quiet_since[name] is None:
                            self.quiet_since[name] = now
                        elif now - self.quiet_since[name] >= 0.25:
                            self.channel_state[name] = "stationary"
                            # Keep output continuous; use the robust center only as
                            # the release reference, never snap output to it.
                            self.anchor[sl] = center[sl]
                            self.quiet_since[name] = None
                    else:
                        self.quiet_since[name] = None
            alphas[name] = alpha
        self.state = "stationary" if all(s == "stationary" for s in self.channel_state.values()) else "moving"
        self.last_time = now
        self.last_raw_pq = raw.copy()
        self._log(mode, raw, errors["position"], errors["rotation"], alphas["position"], alphas["rotation"])
        return pt.transform_from_pq(self.output_pq.copy())

    def _log(self, mode, raw_pq, position_error, rotation_error,
             position_alpha, rotation_alpha):
        state_key = (mode, self.channel_state['position'], self.channel_state['rotation'])
        if state_key == self.last_logged_state:
            return
        self.last_logged_state = state_key
        pose_filter_logger.info(json.dumps({
            "timestamp": time.time(),
            "side": self.side,
            "mode": mode,
            "state": self.state,
            "filter_version": 2,
            "position_state": self.channel_state["position"],
            "rotation_state": self.channel_state["rotation"],
            "raw_pq": raw_pq.tolist(),
            "filtered_pq": self.output_pq.tolist(),
            "position_error": position_error,
            "rotation_error": rotation_error,
            "position_alpha": position_alpha,
            "rotation_alpha": rotation_alpha,
        }, separators=(",", ":")))

class Teleopoperator:
    def __init__(self):        
        self.webserver = WebServer()
        self.webserver.on_hand = self.hand_cb
        self.webserver.on_pedal = self.pedal_cb
        self.webserver.on_control = self.control_cb
        self.webserver.on_remote = self.remote_cb
        
        self.on_pub_goal = None
        self.on_pub_gripper = None
        self.on_pub_head = None
        self.on_cmd_vel = None
        
        self.on_get_current_eef_pose = None
        self.on_get_initial_eef_pose = None
        self.on_reset = None
        self.on_done = None
        self.on_rerecord = None
        
        self.teleop_mode = None
        self.remote_mode = "gripper"
        
        self.percise_mode = True
        self.solve = get_solve(scale=1.0)
        
        self.lift_distance = INITIAL_LIFT_DISTANCE
        self.lift_remote_last_time = None
        self.Tscam = { "left": None, "right": None, }
        self.Tcamgoal_last = { "left": None, "right": None }
        self.pose_filter = {
            "left": PoseMotionFilter("left"),
            "right": PoseMotionFilter("right"),
        }
        self.arm_movement_locked_last = ARM_MOVEMENT_LOCKED.copy()
        self.locked_arm_goal = { "left": None, "right": None }
        
        self.gripper_lock = { "left": False, "right": False }
        self.last_gripper_pos = { "left": GRIPPER_MAX, "right": GRIPPER_MAX }
        
        self.far_seeing = False
        
    async def reset_Tscam(self):
        self.Tscam = { "left": None, "right": None, }
        self.Tcamgoal_last = { "left": None, "right": None }
        for pose_filter in self.pose_filter.values():
            pose_filter.reset()
        
        while True: # wait for new tag result
            ok = True
            for side in ["left", "right"]:
                if self.Tcamgoal_last[side] is None:
                    # logger.info(f"Waiting for new Tcamgoal_last {side}")
                    ok = False
            if ok:
                break
            
            await asyncio.sleep(0.1)

        for side in ["left", "right"]:
            Tsgoal = self.on_get_current_eef_pose(side)
            Tcamgoal = self.Tcamgoal_last[side]
            self.Tscam[side] = Tsgoal @ np.linalg.inv(Tcamgoal)
            logger.info(f"Tscam ({side}): \n{str(self.Tscam[side])}")
                
    async def update_percise_mode(self, percise_mode):
        last_percise_mode = self.percise_mode
        # if last_percise_mode == percise_mode:
        #     return
        self.percise_mode = percise_mode
        self.solve = get_solve(scale=0.6 if self.percise_mode == "more_percise" else 1.3) # scale means to amplify motion
        await self.reset_Tscam()
        
    def update_teleop_mode(self, teleop_mode):
        self.teleop_mode = teleop_mode
        assert self.teleop_mode in ["on", None]
        if self.teleop_mode == "on":
            if self.percise_mode == "more_percise":
                self.webserver.control_datachannel_log("Teleop Mode: On (More Percise)")
                logger.info("Teleop Mode: On (More Percise)")
            elif self.percise_mode:
                self.webserver.control_datachannel_log("Teleop Mode: On (Percise)")
                logger.info("Teleop Mode: On (Percise)")
            else:
                self.webserver.control_datachannel_log("Teleop Mode: On")
                logger.info("Teleop Mode: On")
        else:
            self.webserver.control_datachannel_log("Teleop Mode: Off")
            logger.info("Teleop Mode: Off")

    async def reset_arm(self, lift_distance=INITIAL_LIFT_DISTANCE, joint_bent=math.pi/4, far_seeing=False):  
        self.far_seeing = far_seeing
        self.lift_distance = lift_distance
        self.lift_remote_last_time = None
        goal_pose = {
            "left": self.on_get_initial_eef_pose("left", [self.lift_distance, joint_bent, -joint_bent, 0, 0, 0]),
            "right": self.on_get_initial_eef_pose("right", [self.lift_distance, -joint_bent, joint_bent, 0, 0, 0]),
        }

        while True:
            ok = True
            for side in ["left", "right"]:
                curr_pose = self.on_get_current_eef_pose(side)
                
                goal_pose_pq = pt.pq_from_transform(goal_pose[side])
                curr_pose_pq = pt.pq_from_transform(curr_pose)
                
                pos_dist = math.dist(goal_pose_pq[:3], curr_pose_pq[:3])
                rot_dist = pr.quaternion_dist(
                    goal_pose_pq[3:],
                    curr_pose_pq[3:]
                )
            
                if not (pos_dist < 0.02 and rot_dist < 0.03):
                    # logger.info(f"Resetting {side}: pos_dist {pos_dist}m, rot_dist {rot_dist}rad, curr_pose: \n{curr_pose}")
                    ok = False            
            if ok:
                break
            
            for side in ["left", "right"]:
                self.on_pub_goal(side, goal_pose[side])
                self.on_pub_gripper(side, self.last_gripper_pos[side])
            
            if self.far_seeing:
                self.on_pub_head(0, FAR_SEEING_HEAD_TILT)
            else:
                self.on_pub_head(0, self.get_head_tilt(self.lift_distance))
            
            await asyncio.sleep(0.1)

        # A locked arm must hold the reset result, rather than returning to the
        # pose that was captured before reset.
        for side in ["left", "right"]:
            if ARM_MOVEMENT_LOCKED[side]:
                self.locked_arm_goal[side] = goal_pose[side].copy()

    def get_head_tilt(self, lift_distance):
        point0_lift = 0
        point0_tilt = 1.36
        point1_lift = 0.8
        point1_tilt = 1.06
        
        return point0_tilt + (point1_tilt - point0_tilt) * (lift_distance - point0_lift) / (point1_lift - point0_lift)

    def hand_cb(self, camera_matrix, distortion_coefficients, corners, ids):
        Tcamgoal = {}

        Tcamgoal["left"], Tcamgoal["right"] = self.solve(
            camera_matrix, distortion_coefficients,
            aruco_corners=corners, aruco_ids=ids,
            debug=False,
            debug_image=None,
        ) # 1ms@1080p
        
        for side in ["left", "right"]:
            if Tcamgoal[side] is not None:
                Tcamgoal[side] = self.pose_filter[side].update(
                    Tcamgoal[side], self.percise_mode
                )
                self.Tcamgoal_last[side] = Tcamgoal[side]

        if self.teleop_mode is not None:
            for side in ["left", "right"]:
                if self.Tcamgoal_last[side] is None:
                    self.webserver.control_datachannel_log(f"Connect capture and making sure {side} are in the camera view!")
                    raise Exception(f"Connect capture and making sure {side} are in the camera view!")

            for side in ["left", "right"]:
                if self.Tscam[side] is None:
                    self.webserver.control_datachannel_log(f"Reset {side} arm first!")
                    raise Exception(f"Reset {side} arm first!")
            
            for side in ["left", "right"]:
            # for side in ["left"]:
                locked = ARM_MOVEMENT_LOCKED[side]
                was_locked = self.arm_movement_locked_last[side]
                self.arm_movement_locked_last[side] = locked
                if locked:
                    # Capture the actual arm pose once, then keep publishing
                    # that fixed goal. This ignores all handle motion while
                    # still providing a complete command stream to IK and the
                    # dataset recorder.
                    if not was_locked or self.locked_arm_goal[side] is None:
                        self.locked_arm_goal[side] = self.on_get_current_eef_pose(side).copy()
                        logger.info(f"{side.capitalize()} Arm Movement Lock: Locked at current pose")
                        self.webserver.control_datachannel_log(
                            f"{side.capitalize()} Arm Movement Lock: Locked at current pose"
                        )
                    Tsgoal = self.locked_arm_goal[side]
                    self.on_pub_goal(
                        side, Tsgoal, Tscam=self.Tscam[side], Tsgoal_inactive=Tsgoal
                    )
                    continue

                if was_locked:
                    # The handle may have moved while locked. Re-anchor it to
                    # the current arm pose so unlocking cannot cause a jump.
                    Tsgoal = self.on_get_current_eef_pose(side)
                    self.Tscam[side] = Tsgoal @ np.linalg.inv(self.Tcamgoal_last[side])
                    logger.info(f"{side.capitalize()} Arm Movement Lock: Unlocked and re-anchored")
                    self.webserver.control_datachannel_log(
                        f"{side.capitalize()} Arm Movement Lock: Unlocked and re-anchored"
                    )
                    self.locked_arm_goal[side] = None

                Tsgoal = self.Tscam[side] @ self.Tcamgoal_last[side]
                self.on_pub_goal(side, Tsgoal, Tscam=self.Tscam[side], Tsgoal_inactive=Tsgoal)
        
            if self.far_seeing:
                self.on_pub_head(0, FAR_SEEING_HEAD_TILT)
            else:
                self.on_pub_head(0, self.get_head_tilt(self.lift_distance))
        
    def pedal_cb(self, pedal_real_values):
        pedal_names = ["angular-pos", "angular-neg", "linear-neg", "linear-pos"]
        non_sensetive_area = 0.1
        cliped_pedal_real_values = np.clip((np.array(pedal_real_values) - 0.5) / (0.5 - non_sensetive_area) * 0.5 + 0.5, 0, 1)
        
        # Debug Log: Print values periodically to check input range
        # if np.random.rand() < 0.05: # approx every 20 calls
            # logger.info(f"Pedal Raw: {np.round(pedal_real_values, 2)} -> Clipped: {np.round(cliped_pedal_real_values, 2)}")
            
        # if self.teleop_mode == "arm":
        # elif self.teleop_mode == "base":
        values = dict(zip(pedal_names, cliped_pedal_real_values))

        LINEAR_VEL_MAX = 1
        ANGULAR_VEL_MAX = 1
        linear_vel = (values["linear-pos"] - values["linear-neg"]) * LINEAR_VEL_MAX
        angular_vel = (values["angular-pos"] - values["angular-neg"]) * ANGULAR_VEL_MAX * (-1 if linear_vel < 0 else 1)

        self.on_cmd_vel(linear_vel, angular_vel)

    def remote_cb(self, remote_values):
        side = remote_values["side"]
        force = remote_values.get("force", 0.0)
        # event_code = remote_values.get("eventCode", 0)
        # button_id = remote_values.get("buttonId", 0)
        
        if self.teleop_mode is None:
            self.lift_remote_last_time = None
            return

        if side not in ["left", "right"]:
            return
        
        if self.remote_mode == "lift":
            # 右升左降
            LIFT_VEL_MAX = 0
            if self.percise_mode == True:
                LIFT_VEL_MAX = 0.25
            elif self.percise_mode == False:
                LIFT_VEL_MAX = 0.35
            else:
                LIFT_VEL_MAX = 0.18
            lift_vel = force * LIFT_VEL_MAX * (1 if side == "right" else -1)

            now = time.monotonic()
            TIME_DELTA = 0.0 if self.lift_remote_last_time is None else now - self.lift_remote_last_time
            self.lift_remote_last_time = now
            change = lift_vel * TIME_DELTA
            # change_left = 1
            # change_right = 1
            LIFT_DISTANCE_MIN = 0.18
            LIFT_DISTANCE_MAX = 1.18
            if self.lift_distance + change < LIFT_DISTANCE_MIN or self.lift_distance + change > LIFT_DISTANCE_MAX:
                # change_left = 0 if self.lift_distance + change < LIFT_DISTANCE_MIN else 1
                # change_right = 0 if self.lift_distance + change > LIFT_DISTANCE_MAX else 1
                logger.warn("Lift Over Limit")
                self.webserver.control_datachannel_log("Lift Over Limit")
            elif change:
                self.Tscam["left"][2,3] += change # *change_left
                self.Tscam["right"][2,3] += change # *change_right
                self.lift_distance += change
                logger.info(f"Lift Distance: {self.lift_distance:.3f}")
                self.webserver.control_datachannel_log(f"Lift Distance: {self.lift_distance:.3f}")
        elif self.remote_mode == "gripper":
            self.lift_remote_last_time = None
            gripper_pos = (1 - force) * GRIPPER_MAX
            
            if self.gripper_lock[side] == True and gripper_pos > GRIPPER_MAX * 0.9:
                self.gripper_lock[side] = 'ready_to_unlock'
                
                logger.info(f"{side.capitalize()} Gripper Lock: Locked (Ready to Unlock)")
                self.webserver.control_datachannel_log(f"{side.capitalize()} Gripper Lock: Locked (Ready to Unlock)")
                
            if self.gripper_lock[side] == 'ready_to_unlock' and gripper_pos <= self.last_gripper_pos[side]:
                self.gripper_lock[side] = False
                logger.info(f"{side.capitalize()} Gripper Lock: Unlocked")
                self.webserver.control_datachannel_log(f"{side.capitalize()} Gripper Lock: Unlocked")
            
            if self.gripper_lock[side] == False:
                self.last_gripper_pos[side] = gripper_pos
                self.on_pub_gripper(side, self.last_gripper_pos[side])
                
        else:
            logger.warn(f"Unknown remote mode: {self.remote_mode}")
            self.webserver.control_datachannel_log(f"Unknown remote mode: {self.remote_mode}")
    
    async def control_cb(self, control_type):
        self.webserver.control_datachannel_log(f"Cmd: {control_type}")
        logger.info(f"Cmd: {control_type}")
        if control_type == "operate_reset":
            self.update_teleop_mode(None)
            self.last_gripper_pos = { "left": GRIPPER_MAX, "right": GRIPPER_MAX }
            await self.reset_arm(INITIAL_LIFT_DISTANCE, math.pi/4, far_seeing=False)

            # # uncomment when collecting datas to avoid stair in the dataset
            # await self.update_percise_mode(percise_mode=True) 
            # self.update_teleop_mode("arm")
            
            self.on_reset()
            self.webserver.control_datachannel_log("Reset event")
            logger.info("Reset event")
        elif control_type == "done":
            self.on_done()
            self.webserver.control_datachannel_log("Done event")
            logger.info("Done event")
        elif control_type == "rerecord":
            self.on_rerecord()
            self.webserver.control_datachannel_log("Rerecord event")
            logger.info("Rerecord event")
        elif control_type == "teleop_mode_off":
            self.update_teleop_mode(None)
        elif control_type == "teleop_mode_on":
            self.lift_remote_last_time = None
            await self.update_percise_mode(percise_mode=True)
            self.update_teleop_mode("on")
        elif control_type == "teleop_mode_toggle":
            self.lift_remote_last_time = None
            if self.teleop_mode is None:
                await self.update_percise_mode(percise_mode=True)
                self.update_teleop_mode("on")
            else:
                self.update_teleop_mode(None)
        elif control_type == "move_reset":
            self.update_teleop_mode(None)
            await self.reset_arm(self.lift_distance, math.pi/2*0.9, far_seeing=True)
            await self.update_percise_mode(percise_mode=True)
        elif control_type == "percise_mode_false":
            self.update_teleop_mode(None)
            await self.update_percise_mode(percise_mode=False)
            self.update_teleop_mode("on")
        elif control_type == "percise_mode_true":
            self.update_teleop_mode(None)
            await self.update_percise_mode(percise_mode=True)
            self.update_teleop_mode("on")
        elif control_type == "percise_mode_more_percise":
            self.update_teleop_mode(None)
            await self.update_percise_mode(percise_mode="more_percise")
            self.update_teleop_mode("on")
        elif control_type == "percise_mode_next":
            next_mode_dict = {
                False: True,
                True: "more_percise",
                "more_percise": False,
            }
            self.update_teleop_mode(None)
            await self.update_percise_mode(percise_mode=next_mode_dict[self.percise_mode])
            self.update_teleop_mode("on")
        elif control_type == "gripper_lock_left":
            self.gripper_lock["left"] = True
            logger.info("Left Gripper Lock: Locked, release your pedal to unlock")
            self.webserver.control_datachannel_log("Left Gripper Lock: Locked, release your pedal to unlock")
        elif control_type == "gripper_lock_right":
            self.gripper_lock["right"] = True
            logger.info("Right Gripper Lock: Locked, release your pedal to unlock")
            self.webserver.control_datachannel_log("Right Gripper Lock: Locked, release your pedal to unlock")
        elif control_type == "gripper_mode":
            self.remote_mode = "gripper"
            self.lift_remote_last_time = None
            logger.info("Change to Gripper Mode")
            self.webserver.control_datachannel_log("Change to Gripper Mode")
        elif control_type == "lift_mode":
            self.remote_mode = "lift"
            self.lift_remote_last_time = None
            logger.info("Change to Lift Mode")
            self.webserver.control_datachannel_log("Change to Lift Mode")
        elif control_type == "remote_mode_toggle":
            self.remote_mode = "lift" if self.remote_mode == "gripper" else "gripper"
            self.lift_remote_last_time = None
            if self.remote_mode == "gripper":
                logger.info("Change to Gripper Mode")
                self.webserver.control_datachannel_log("Change to Gripper Mode")
            else:
                logger.info("Change to Lift Mode")
                self.webserver.control_datachannel_log("Change to Lift Mode")
            
    def error_cb(self, msg):
        self.webserver.loop.call_soon_threadsafe(self.webserver.control_datachannel_log, msg)
