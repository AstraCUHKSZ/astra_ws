#!/usr/bin/env python3
"""逐条给评测 episode 记结果，写到 eval_notes/<数据集名>.csv。只依赖 pandas。

    python tools/annotate_eval.py <数据集目录> [--notes-dir eval_notes] [--scene "..."] [--all]

默认只问还没标注过的 episode。每条问三样（直接回车 = 跳过这一条，之后可以再跑补上）:
  outcome  s=成功  f=失败  a=中止/作废
  reached  夹爪第一次去了哪: l=左瓶 m=中瓶 r=右瓶 lm=左中之间 mr=中右之间 n=哪都没去 ?=不确定
  grasped  有没有夹到任何东西（不管对不对）: y / n
另外可以写瓶子位置（网格坐标，自由格式）和备注。

列的含义和离线分析里的 labels.csv 对应，analysis/ingest.py 会自动读这张表。
"""
import argparse
import sys
from pathlib import Path

import pandas as pd

COLS = ["episode", "prompt", "length", "outcome", "reached", "grasped", "scene", "bottles", "note"]
OUTCOME = {"s": "success", "f": "fail", "a": "aborted"}
REACHED = {"l": "left", "m": "middle", "r": "right", "lm": "between_lm", "mr": "between_mr", "n": "none", "?": ""}


def load_episodes(root: Path) -> pd.DataFrame:
    ep = pd.concat(pd.read_parquet(f) for f in sorted((root / "meta" / "episodes").glob("*/*.parquet")))
    ep = ep.sort_values("episode_index")
    return pd.DataFrame({"episode": ep["episode_index"].astype(int).values,
                         "prompt": [" | ".join(t) for t in ep["tasks"]],
                         "length": ep["length"].astype(int).values})


def ask(label: str, choices: dict | None = None, default: str = "") -> str | None:
    """返回 None 表示用户想跳过这一条。"""
    while True:
        hint = f" [{'/'.join(choices)}]" if choices else ""
        dflt = f" (回车={default})" if default else ""
        raw = input(f"  {label}{hint}{dflt}: ").strip()
        if raw == "" and default:
            return choices.get(default, default) if choices else default
        if choices is None:
            return raw
        if raw == "":
            return None
        if raw in choices:
            return choices[raw]
        print(f"    请输入 {', '.join(choices)} 之一")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--notes-dir", type=Path, default=Path(__file__).resolve().parent.parent / "eval_notes")
    ap.add_argument("--scene", default="", help="这批 episode 的场景说明，作为默认值")
    ap.add_argument("--all", action="store_true", help="重新问所有 episode（包括标过的）")
    args = ap.parse_args()

    root = args.dataset.resolve()
    eps = load_episodes(root)
    out = args.notes_dir / f"{root.name}.csv"
    args.notes_dir.mkdir(parents=True, exist_ok=True)
    notes = pd.read_csv(out, dtype=str).fillna("") if out.exists() else pd.DataFrame(columns=COLS)
    notes["episode"] = notes["episode"].astype(int) if len(notes) else notes["episode"]
    done = set() if args.all else set(notes.loc[notes["outcome"] != "", "episode"])
    todo = eps[~eps.episode.isin(done)]
    if todo.empty:
        print(f"{root.name}: 全部 {len(eps)} 条都已标注 -> {out}")
        return

    print(f"{root.name}: {len(todo)} 条待标注（共 {len(eps)} 条）。outcome 直接回车 = 跳过；Ctrl-C = 保存并退出\n")
    last = {"scene": args.scene, "bottles": ""}
    rows = {int(r.episode): r.to_dict() for _, r in notes.iterrows()}
    try:
        for r in todo.itertuples():
            print(f"episode {r.episode}  ({r.length} 帧, {r.length / 30:.1f}s)  prompt: {r.prompt}")
            outcome = ask("outcome  s成功 f失败 a中止", OUTCOME)
            if outcome is None:
                print("    跳过\n")
                continue
            reached = ask("reached  l左 m中 r右 lm mr n没去 ?不确定", REACHED)
            grasped = ask("grasped  夹到任何东西", {"y": "yes", "n": "no"},
                          default="y" if outcome == "success" else "")
            last["scene"] = ask("scene", default=last["scene"]) or ""
            last["bottles"] = ask("bottles 瓶子位置（网格坐标）", default=last["bottles"]) or ""
            note = ask("note") or ""
            rows[r.episode] = {"episode": r.episode, "prompt": r.prompt, "length": r.length, "outcome": outcome,
                               "reached": reached or "", "grasped": grasped or "", "scene": last["scene"],
                               "bottles": last["bottles"], "note": note}
            print()
    except (KeyboardInterrupt, EOFError):
        print("\n中断，保存已填的部分。")
    if rows:
        pd.DataFrame([rows[k] for k in sorted(rows)], columns=COLS).to_csv(out, index=False)
        print(f"已保存 {len(rows)} 条 -> {out}")


if __name__ == "__main__":
    sys.exit(main())
