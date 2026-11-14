#!/usr/bin/env python3
"""开机自检：用一条刚录的短 episode 检查相机是否对焦、顶部相机有没有被碰歪、瓶子是否摆在参考位置。

先随便录一条几秒钟的 episode（机器人不用动，三个瓶子摆在标准位置），然后:

    python tools/check_setup.py <数据集目录> --save-reference     # 第一次：把这一帧存成参考
    python tools/check_setup.py <数据集目录>                      # 之后每次开机：和参考比

默认取数据集最后一条 episode 的第 0 帧。参考存在 eval_notes/setup_reference/。
只依赖 numpy、opencv、av、pandas（都在 lerobot 的 venv 里）。
"""
import argparse
import json
from pathlib import Path

import av
import cv2
import numpy as np
import pandas as pd

CAMS = ["head", "wrist_left", "wrist_right"]
FPS = 30
SHARP_WARN = {"head": 2.5, "wrist_left": 25.0, "wrist_right": 25.0}   # 旧数据: 正常腕部约 50-100，失焦约 5
CAM_SHIFT_WARN_PX = 4.0
CAP_SHIFT_WARN_PX = 8.0     # 约 1 cm


def first_frame(root: Path, cam: str, episode: int) -> np.ndarray:
    ep = pd.concat(pd.read_parquet(f) for f in sorted((root / "meta" / "episodes").glob("*/*.parquet")))
    r = ep[ep["episode_index"] == episode].iloc[0]
    key = f"observation.images.{cam}"
    path = (root / "videos" / key / f"chunk-{int(r[f'videos/{key}/chunk_index']):03d}"
            / f"file-{int(r[f'videos/{key}/file_index']):03d}.mp4")
    t0 = float(r[f"videos/{key}/from_timestamp"])
    with av.open(str(path)) as c:
        st = c.streams.video[0]
        tb = float(st.time_base)
        c.seek(int(max(t0 - 0.5, 0) / tb), stream=st, backward=True)
        for fr in c.decode(st):
            if round(fr.pts * tb * FPS) >= round(t0 * FPS):
                return fr.to_ndarray(format="rgb24")
    raise RuntimeError(f"cannot decode {path}")


def sharpness(img: np.ndarray) -> float:
    return float(cv2.Laplacian(img.astype(np.float32).mean(-1), cv2.CV_32F).var())


def find_caps(img: np.ndarray) -> list[list[float]]:
    """顶部相机里红瓶盖的中心，从左到右。"""
    r, g, b = (img[..., i].astype(int) for i in range(3))
    mask = ((r > 150) & (g < 100) & (b < 100) & (r - np.maximum(g, b) > 80)).astype(np.uint8)
    mask[img.shape[0] * 5 // 12:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
    return sorted([[float(cent[i][0]), float(cent[i][1])] for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 150])


def camera_shift(ref: np.ndarray, cur: np.ndarray) -> tuple[float, float]:
    """顶部相机整体平移了多少像素。只用画面下半部（桌面和机器人本体），避开会被挪动的瓶子。"""
    def prep(x):
        g = cv2.cvtColor(x, cv2.COLOR_RGB2GRAY).astype(np.float32)
        return cv2.Sobel(g, cv2.CV_32F, 1, 0) ** 2 + cv2.Sobel(g, cv2.CV_32F, 0, 1) ** 2
    a, b = prep(ref), prep(cur)
    h = a.shape[0]
    a[h // 6: h * 5 // 8] = 0
    b[h // 6: h * 5 // 8] = 0
    (dx, dy), _ = cv2.phaseCorrelate(a, b)
    return float(dx), float(dy)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--episode", type=int, default=None, help="默认最后一条")
    ap.add_argument("--save-reference", action="store_true")
    ap.add_argument("--ref-dir", type=Path,
                    default=Path(__file__).resolve().parent.parent / "eval_notes" / "setup_reference")
    args = ap.parse_args()
    root = args.dataset.resolve()
    n = json.load(open(root / "meta" / "info.json"))["total_episodes"]
    ep = n - 1 if args.episode is None else args.episode
    imgs = {c: first_frame(root, c, ep) for c in CAMS}
    sharp = {c: sharpness(imgs[c]) for c in CAMS}
    caps = find_caps(imgs["head"])
    print(f"{root.name} episode {ep}")

    ok = True
    for c in CAMS:
        bad = sharp[c] < SHARP_WARN[c]
        ok &= not bad
        print(f"  [{'!!' if bad else 'ok'}] 清晰度 {c:12s} {sharp[c]:7.1f}" + ("   <- 可能失焦" if bad else ""))
    print(f"  [{'ok' if len(caps) == 3 else '!!'}] 顶部相机检测到 {len(caps)} 个瓶盖: "
          + ", ".join(f"({u:.0f},{v:.0f})" for u, v in caps))
    ok &= len(caps) == 3

    args.ref_dir.mkdir(parents=True, exist_ok=True)
    ref_json = args.ref_dir / "reference.json"
    if args.save_reference:
        for c in CAMS:
            cv2.imwrite(str(args.ref_dir / f"{c}.png"), cv2.cvtColor(imgs[c], cv2.COLOR_RGB2BGR))
        json.dump({"dataset": root.name, "episode": ep, "caps": caps, "sharpness": sharp}, open(ref_json, "w"), indent=1)
        print(f"  已存为参考 -> {args.ref_dir}")
        return 0
    if not ref_json.exists():
        print("  没有参考。第一次请加 --save-reference。")
        return 1

    ref = json.load(open(ref_json))
    ref_head = cv2.cvtColor(cv2.imread(str(args.ref_dir / "head.png")), cv2.COLOR_BGR2RGB)
    dx, dy = camera_shift(ref_head, imgs["head"])
    bad = max(abs(dx), abs(dy)) > CAM_SHIFT_WARN_PX
    ok &= not bad
    print(f"  [{'!!' if bad else 'ok'}] 顶部相机相对参考平移 dx={dx:+.1f} dy={dy:+.1f} px"
          + ("   <- 相机可能被碰过" if bad else ""))
    if len(caps) == len(ref["caps"]):
        for name, (u, v), (ru, rv) in zip(["左", "中", "右"], caps, ref["caps"]):
            # 扣掉相机整体平移后，瓶盖相对参考的位置
            du, dv = u - ru - dx, v - rv - dy
            bad = max(abs(du), abs(dv)) > CAP_SHIFT_WARN_PX
            ok &= not bad
            print(f"  [{'!!' if bad else 'ok'}] {name}瓶瓶盖相对参考 du={du:+.1f} dv={dv:+.1f} px"
                  f"（约 {du * 0.12:+.1f} cm 横向）" + ("   <- 瓶子没摆在参考位置" if bad else ""))
    for c in CAMS:
        r = ref["sharpness"][c]
        if sharp[c] < 0.5 * r:
            ok = False
            print(f"  [!!] {c} 清晰度只有参考的 {sharp[c] / r:.0%}")
    print("  结论:", "通过" if ok else "有问题，见上面带 !! 的行")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
