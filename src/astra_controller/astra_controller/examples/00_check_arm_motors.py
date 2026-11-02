from astra_controller.arm_controller import ArmController
import argparse
import time
import math


def is_finite_list(xs):
    return all((x is not None) and math.isfinite(float(x)) for x in xs)


def main():
    parser = argparse.ArgumentParser(
        description="Arm motor health check: reads feedback for all joints and prints a short summary.\n"
        "Tip: run with torque=0 and move joints by hand; each joint position should change."
    )
    parser.add_argument("--device", default="/dev/tty_puppet_left")
    parser.add_argument("--torque", type=int, default=0, help="0 disables torque; 1/128 enables")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.2)
    parser.add_argument("--move-threshold", type=float, default=0.02, help="rad; used for end summary")
    args = parser.parse_args()

    arm = ArmController(args.device)
    try:
        arm.print_device_output = False
        arm.set_torque(int(args.torque))

        # Wait for first feedback frame
        t0 = time.monotonic()
        pos0 = None
        while time.monotonic() - t0 < 5.0:
            pos, vel, eff, ts = arm.get_pos()
            if pos is not None:
                pos0 = pos
                break
            time.sleep(0.05)

        if pos0 is None:
            raise RuntimeError("No feedback received within 5s (check wiring / firmware / port)")

        min_pos = pos0.copy()
        max_pos = pos0.copy()
        last_ts = None
        frames = 0

        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            pos, vel, eff, ts = arm.get_pos()
            if pos is None:
                time.sleep(args.period)
                continue

            frames += 1
            for i in range(len(pos)):
                min_pos[i] = min(min_pos[i], pos[i])
                max_pos[i] = max(max_pos[i], pos[i])

            if last_ts is None or ts != last_ts:
                print(f"t={ts:.3f} pos(rad)={[float(x) for x in pos]}")
                last_ts = ts

            time.sleep(args.period)

        print("\n=== Summary ===")
        print(f"device={args.device} frames={frames}")
        ok = is_finite_list(min_pos) and is_finite_list(max_pos)
        print(f"feedback_finite={ok}")

        moved = []
        for i in range(len(min_pos)):
            span = float(max_pos[i] - min_pos[i])
            moved.append(span)
            print(f"joint[{i}] span(rad)={span:.4f} (min={float(min_pos[i]):.4f}, max={float(max_pos[i]):.4f})")

        # Heuristic: if torque=0 and you manually move joints, span should exceed threshold.
        stuck = [i for i, span in enumerate(moved) if span < args.move_threshold]
        if stuck:
            print(
                "possible_stuck_joints="
                + str(stuck)
                + " (if you did not move them, this is expected)"
            )
        else:
            print("all_joints_moved_over_threshold=true")

    finally:
        arm.stop()


if __name__ == "__main__":
    main()
