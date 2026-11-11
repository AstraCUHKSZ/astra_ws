"""升降机初始化工具。

python3 00_lift_control.py set 0.775              # 左右都把当前位置设为 0.775 m
python3 00_lift_control.py set 0.775 --side left
python3 00_lift_control.py home                   # 向光电门方向找光电门并置零
"""
import argparse
import sys
import time

from astra_controller.lift_controller import LiftController

DEVICES = {
    "right": "/dev/tty_puppet_lift_right",
    "left": "/dev/tty_puppet_lift_left",
}

HOME_TIMEOUT_S = 30  # 半程 600mm @ 100mm/s ≈ 6s，留足余量


def wait(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_set = sub.add_parser("set", help="把当前位置设为指定高度(米)")
    p_set.add_argument("pos", type=float)
    sub.add_parser("home", help="找光电门并置零")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    args = parser.parse_args()

    sides = list(DEVICES) if args.side == "both" else [args.side]
    ctrls = {s: LiftController(DEVICES[s]) for s in sides}

    ok = True
    if args.cmd == "set":
        for c in ctrls.values():
            c.set_origin(args.pos)
        for s, c in ctrls.items():
            done = wait(lambda: c.initialized and abs(c.get_pos()[0] - args.pos) < 0.002, 2)
            print(f"[{s}] set to {args.pos} m: {'OK' if done else 'FAILED'}, pos={c.get_pos()[0]:.4f}")
            ok &= done
    else:
        for c in ctrls.values():
            c.home()
        for s, c in ctrls.items():
            time.sleep(0.2)  # 等第一帧 homing 状态
            wait(lambda: not c.is_homing, HOME_TIMEOUT_S)
            pos = c.get_pos()[0]
            done = c.initialized and not c.is_homing
            print(f"[{s}] home: {'OK' if done else 'FAILED (没找到光电门)'}, pos={pos:.4f}")
            ok &= done

    for c in ctrls.values():
        c.stop()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
