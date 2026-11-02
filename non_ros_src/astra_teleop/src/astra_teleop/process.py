import cv2
import numpy as np
from pathlib import Path
import glob
import yaml
from yaml.loader import SafeLoader
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
from .cam import open_cam
import argparse
from pprint import pprint
import time
import json
import logging

MARKER_LOG_PATH = "/tmp/astra_teleop_marker_selection.jsonl"
marker_logger = logging.getLogger("astra_teleop.marker_selection")
if not marker_logger.handlers:
    marker_log_handler = logging.FileHandler(MARKER_LOG_PATH)
    marker_log_handler.setFormatter(logging.Formatter("%(message)s"))
    marker_logger.addHandler(marker_log_handler)
    marker_logger.setLevel(logging.INFO)
    marker_logger.propagate = False

def calibration_load(calibration_directory="./calibration_images"):
    file_names = glob.glob(str(Path(calibration_directory) / 'calibration_results_*.yaml'))
    file_names.sort()
    assert len(file_names) > 0, 'Webcam: No camera calibration files found.'

    file_name = file_names[-1]
    with open(file_name) as f:
        camera_calibration = yaml.load(f, Loader=SafeLoader)
    assert camera_calibration, 'Webcam: Failed to successfully load camera calibration results.'

    print('Webcam: Loaded camera calibration results from file =', file_name)
    print('Webcam: Loaded camera calibration results =', camera_calibration)
    return np.array(camera_calibration['camera_matrix']), np.array(camera_calibration['distortion_coefficients'])

def transform_from_rvec_tvec(rvec, tvec):
    # cv2.Rodrigues(rvec.squeeze())[0] == pr.matrix_from_compact_axis_angle(rvec.squeeze())
    return pt.transform_from(
        pr.matrix_from_compact_axis_angle(rvec), 
        tvec
    )

def rvec_tvec_from_transform(transform):
    rvec = pr.compact_axis_angle_from_matrix(transform[:3,:3])
    tvec = transform[:3,3]
    return rvec, tvec

def get_detect():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_detection_parameters = cv2.aruco.DetectorParameters()
    # aruco_detection_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # ~30ms
    # aruco_detection_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG # Most accurate but also slowest with ~200-300ms
    # aruco_detection_parameters.aprilTagQuadDecimate = 2
    # aruco_detection_parameters.cornerRefinementWinSize = 2
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_detection_parameters)
    
    def detect(
        rgb_image,
        debug,
        debug_image
    ):
        aruco_corners, aruco_ids, aruco_rejected_image_points = detector.detectMarkers(rgb_image)

        if debug:
            cv2.aruco.drawDetectedMarkers(debug_image, aruco_corners, aruco_ids)

            # # Rejected
            # cv2.aruco.drawDetectedMarkers(debug_image, aruco_rejected_image_points, None, (100, 0, 255))

            # tag_transforms = []
            
        return aruco_corners, aruco_ids
    return detect

def get_solve(scale=1):
    # PnP is solved frame-by-frame, but the handle motion is continuous. Keep the
    # last physically valid solution both as the next solver's initial guess and
    # as a guard against jumping to a different PnP solution branch.
    last_valid_pose = {
        "left": None,
        "right": None,
    }
    max_translation_speed = 2.0  # m/s
    max_rotation_speed = 4 * np.pi  # rad/s (720 deg/s)
    translation_tolerance = 0.03  # m, allows for measurement noise
    rotation_tolerance = np.deg2rad(15)  # rad
    continuity_timeout = 0.5  # s; allow reacquisition after a long dropout

    # set coordinate system
    # Coordinate setting: 
    # https://stackoverflow.com/questions/53277597/fundamental-understanding-of-tvecs-rvecs-in-opencv-aruco
    obj_points_map = { # right part
        # top_left, top_right, bottom_right, bottom_left
        0:  [ (-15.00, -15.00,  48.28), (-15.00,  15.00,  48.28), ( 15.00,  15.00,  48.28), ( 15.00, -15.00,  48.28) ],
        
        1:  [ ( 23.54, -15.00,  44.75), ( 23.54,  15.00,  44.75), ( 44.75,  15.00,  23.54), ( 44.75, -15.00,  23.54) ],
        2:  [ (-15.00, -23.54,  44.75), ( 15.00, -23.54,  44.75), ( 15.00, -44.75,  23.54), (-15.00, -44.75,  23.54) ],
        3:  [ (-23.54,  15.00,  44.75), (-23.54, -15.00,  44.75), (-44.75, -15.00,  23.54), (-44.75,  15.00,  23.54) ],
        4:  [ ( 15.00,  23.54,  44.75), (-15.00,  23.54,  44.75), (-15.00,  44.75,  23.54), ( 15.00,  44.75,  23.54) ],
        
        5:  [ ( 48.28, -15.00,  15.00), ( 48.28,  15.00,  15.00), ( 48.28,  15.00, -15.00), ( 48.28, -15.00, -15.00) ],
        6:  [ ( 23.54, -44.75,  15.00), ( 44.75, -23.54,  15.00), ( 44.75, -23.54, -15.00), ( 23.54, -44.75, -15.00) ],
        7:  [ (-15.00, -48.28,  15.00), ( 15.00, -48.28,  15.00), ( 15.00, -48.28, -15.00), (-15.00, -48.28, -15.00) ],
        8:  [ (-44.75, -23.54,  15.00), (-23.54, -44.75,  15.00), (-23.54, -44.75, -15.00), (-44.75, -23.54, -15.00) ],
        9:  [ (-48.28,  15.00,  15.00), (-48.28, -15.00,  15.00), (-48.28, -15.00, -15.00), (-48.28,  15.00, -15.00) ],
        10: [ (-23.54,  44.75,  15.00), (-44.75,  23.54,  15.00), (-44.75,  23.54, -15.00), (-23.54,  44.75, -15.00) ],
        11: [ ( 15.00,  48.28,  15.00), (-15.00,  48.28,  15.00), (-15.00,  48.28, -15.00), ( 15.00,  48.28, -15.00) ],
        12: [ ( 44.75,  23.54,  15.00), ( 23.54,  44.75,  15.00), ( 23.54,  44.75, -15.00), ( 44.75,  23.54, -15.00) ],
        
        13: [ ( 44.75, -15.00, -23.54), ( 44.75,  15.00, -23.54), ( 23.54,  15.00, -44.75), ( 23.54, -15.00, -44.75) ],
        14: [ (-15.00, -44.75, -23.54), ( 15.00, -44.75, -23.54), ( 15.00, -23.54, -44.75), (-15.00, -23.54, -44.75) ],
        15: [ (-44.75,  15.00, -23.54), (-44.75, -15.00, -23.54), (-23.54, -15.00, -44.75), (-23.54,  15.00, -44.75) ],
        16: [ ( 15.00,  44.75, -23.54), (-15.00,  44.75, -23.54), (-15.00,  23.54, -44.75), ( 15.00,  23.54, -44.75) ],
    }
    for i in range(18 - 1): # left part
        obj_points_map[i + 18] = obj_points_map[i]
    
    def solve(
        camera_matrix, distortion_coefficients,
        aruco_corners, aruco_ids,
        debug=False,
        debug_image=None,
        min_aruco_thres=4
    ):
        camera_matrix = np.array(camera_matrix, dtype=np.float32)
        distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
        
        if aruco_ids is None:
            aruco_ids = np.array([])
            aruco_corners = np.array([])
        aruco_ids = np.array(aruco_ids)
        aruco_corners = np.array(aruco_corners, dtype=np.float32)
            
        tags = {
            "left": [],
            "right": [],
        }
        
        for aruco_id, aruco_corner in zip(aruco_ids, aruco_corners):
            aruco_id = aruco_id.item()
            aruco_corner = aruco_corner.squeeze() # shape: (4, 2)
            is_left_hand = aruco_id >= 18 # 18~34 is left hand
            
            if is_left_hand:
                tags["left"].append((aruco_id, aruco_corner))
            else:
                tags["right"].append((aruco_id, aruco_corner))

        tag2cam = {
            "left": None,
            "right": None,
        }
        for side in ["left", "right"]:
            tags[side].sort(key=lambda tag: cv2.contourArea(tag[1]), reverse=True)
            sorted_markers = [
                {
                    "id": int(aruco_id),
                    "area": float(cv2.contourArea(aruco_corner)),
                }
                for aruco_id, aruco_corner in tags[side]
            ]
            selected_markers = sorted_markers[:min_aruco_thres]
            log_record = {
                "timestamp": time.time(),
                "side": side,
                "all_ids": [marker["id"] for marker in sorted_markers],
                "sorted_markers": sorted_markers,
                "selected_ids": [marker["id"] for marker in selected_markers],
                "status": "insufficient_markers",
            }
            if len(tags[side]) < min_aruco_thres:
                # print(f"detected less than {min_aruco_thres} aruco tags")
                # Insufficient-marker frames are expected during idle periods.
                continue
            tags[side] = tags[side][:min_aruco_thres] # pick biggest four tag
            # print(tags[side][0][0])
            
            obj_points = []
            img_points = []
            
            for aruco_id, aruco_corner in tags[side]:
                if aruco_id in obj_points_map:
                    obj_points.extend(obj_points_map[aruco_id])
                    img_points.extend(aruco_corner)

            # rvec shape (3, 1)
            # tvec shape (3, 1)
            object_points = np.asarray(obj_points, dtype=np.float64) / 1000 * scale
            image_points = np.asarray(img_points, dtype=np.float64)
            previous = last_valid_pose[side]
            solve_kwargs = {}
            if previous is not None:
                solve_kwargs = {
                    "rvec": previous["rvec"].copy(),
                    "tvec": previous["tvec"].copy(),
                    "useExtrinsicGuess": True,
                    "flags": cv2.SOLVEPNP_ITERATIVE,
                }

            unknown_variable, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                distortion_coefficients,
                **solve_kwargs,
            )

            log_record.update({
                "status": "ok" if unknown_variable else "solve_pnp_failed",
                "rvec": rvec.squeeze().tolist(),
                "tvec": tvec.squeeze().tolist(),
            })

            if unknown_variable:
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                camera_points = (rotation_matrix @ object_points.T + tvec.reshape(3, 1)).T
                min_depth = float(camera_points[:, 2].min())
                log_record["min_depth"] = min_depth

                reject_reason = None
                if min_depth <= 0:
                    reject_reason = "non_positive_depth"

                now = time.monotonic()
                if reject_reason is None and previous is not None:
                    dt = now - previous["time"]
                    log_record["continuity_dt"] = dt
                    if dt <= continuity_timeout:
                        translation_delta = float(np.linalg.norm(tvec.reshape(3) - previous["tvec"].reshape(3)))
                        relative_rotation = rotation_matrix @ previous["rotation_matrix"].T
                        rotation_delta = float(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1)))
                        translation_limit = translation_tolerance + max_translation_speed * dt
                        rotation_limit = rotation_tolerance + max_rotation_speed * dt
                        log_record.update({
                            "translation_delta": translation_delta,
                            "translation_limit": translation_limit,
                            "rotation_delta": rotation_delta,
                            "rotation_limit": rotation_limit,
                        })
                        if translation_delta > translation_limit:
                            reject_reason = "translation_jump"
                        elif rotation_delta > rotation_limit:
                            reject_reason = "rotation_jump"

                if reject_reason is None:
                    last_valid_pose[side] = {
                        "rvec": rvec.copy(),
                        "tvec": tvec.copy(),
                        "rotation_matrix": rotation_matrix,
                        "time": now,
                    }
                else:
                    log_record["status"] = "rejected"
                    log_record["reject_reason"] = reject_reason

            if log_record['status'] != 'ok':
                marker_logger.info(json.dumps(log_record, separators=(",", ":")))

            if log_record["status"] != "ok":
                continue
            
            tag2cam[side] = transform_from_rvec_tvec(rvec.squeeze(), tvec.squeeze())

        if tag2cam["left"] is not None:
            if debug:
                rvec, tvec = rvec_tvec_from_transform(tag2cam["left"])
                cv2.drawFrameAxes(
                    debug_image,
                    camera_matrix, distortion_coefficients,
                    rvec, tvec,
                    50/2, 2
                )
 
        if tag2cam["right"] is not None:
            if debug:
                rvec, tvec = rvec_tvec_from_transform(tag2cam["right"])
                cv2.drawFrameAxes(
                    debug_image,
                    camera_matrix, distortion_coefficients,
                    rvec, tvec,
                    50/2, 2
                )
                
        return tag2cam["left"], tag2cam["right"]
    return solve

def get_process(
    device="/dev/video0", calibration_directory="./calibration_images", 
    debug=False,
):
    # Open camera
    cam = open_cam(device)

    # Load calibration
    camera_matrix, distortion_coefficients = calibration_load(calibration_directory)
    
    detect = get_detect()
    solve = get_solve()

    def process():
        ret, rgb_image = cam.read()

        if debug:
            # draw results
            debug_image = rgb_image.copy()
        else:
            debug_image = None

        t0 = time.perf_counter()
        aruco_corners, aruco_ids = detect(
            rgb_image,
            debug,
            debug_image,
        ) # 10ms@1080p
        t1 = time.perf_counter()

        tag2cam_left, tag2cam_right = solve(
            camera_matrix, distortion_coefficients,
            aruco_corners, aruco_ids,
            debug,
            debug_image,
        ) # 1ms@1080p
        t2 = time.perf_counter()

        if debug:
            # # visualize via matplotlib
            # import matplotlib.pyplot as plt
            # from pytransform3d.plot_utils import make_3d_axis
            # plt.ion()
            # plt.cla()
            # ax = make_3d_axis(ax_s=1, unit="m", n_ticks=6)
            # pt.plot_transform(ax=ax)
            # for tag2cam in transforms:
            #     pprint(tag2cam)
            #     pt.plot_transform(ax, A2B=tag2cam)
            # # plt.show()
            # plt.draw()
            # plt.pause(0.001)

            # debug_image = cv2.flip(debug_image, 1)
            cv2.imshow('Debug Image', debug_image)
            # debug_image2 = cv2.undistort(debug_image, camera_matrix, distortion_coefficients)
            # cv2.imshow('Debug Image 2', debug_image2)
            if (cv2.waitKey(1) == 27): # Must wait, otherwise imshow will show black screen
                raise Exception("Stop")
        
        print(f"detect time: {t1 - t0}")
        print(f"solve time: {t2 - t1}")
        print(f"total time: {t2 - t0}")
        
        return tag2cam_left, tag2cam_right
        
    return process

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", help="Device name.", default="/dev/video0")
    parser.add_argument("-c", "--calibration_directory", help="Calibration directory.", default="./calibration_images")
    args = parser.parse_args()

    process = get_process(args.device, args.calibration_directory, debug=True)
    while True:
        tag2cam_left, tag2cam_right = process()
        pprint(tag2cam_left)
        pprint(tag2cam_right)
