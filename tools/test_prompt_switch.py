"""Offline test of mid-episode prompt switching (no robot, no model).

Run from astra_ws/non_ros_src/lerobot_new:
    .venv/bin/python ../../tools/test_prompt_switch.py

Checks:
  1. PromptController: schedule, per-episode list, hotkeys.
  2. AsyncPolicyActionRunner.set_task with a fake policy that takes LATENCY seconds per chunk:
     - no flush: old-prompt actions keep flowing during inference, then the new prompt takes over;
       a chunk that was in flight with the old prompt is discarded
     - flush: the next action already comes from the new prompt
     - no deadlock when the switch lands while the queue is empty
  3. A LeRobot dataset accepts a different task string per frame inside one episode.
"""
import tempfile
import time
from pathlib import Path

import numpy as np

from lerobot.scripts import lerobot_record as R
from lerobot.scripts.prompt_schedule import PromptController

FPS, LATENCY, CHUNK = 30, 0.20, 50


class FakeRunner(R.AsyncPolicyActionRunner):
    """Real queue/threading logic; the model call is replaced by a sleep."""

    def __init__(self, task):
        self.calls = []
        super().__init__(policy=_Cfg(), preprocessor=None, postprocessor=None, dataset=None,
                         single_task=task, robot_type="test")

    def _compute_actions(self, observation_frame, task):
        self.calls.append(task)
        time.sleep(LATENCY)
        return [{"task": task, "obs_frame": observation_frame["i"], "k": k} for k in range(CHUNK)]


class _Cfg:
    class config:
        device = "cpu"
        use_amp = False


def rollout(flush, switch_frame, n=150, threshold=8, new="B"):
    r = FakeRunner("A")
    served = []
    for i in range(n):
        t0 = time.perf_counter()
        obs = {"i": i}
        if i == switch_frame:
            r.set_task(new, obs, flush=flush)
        if r.queue_size() <= threshold and not r.has_pending_request():
            r.submit(obs)
        served.append(r.get_next(obs)["task"])
        time.sleep(max(0, 1 / FPS - (time.perf_counter() - t0)))
    r.stop()
    return served, r.calls


def test_controller():
    c = PromptController.from_args("A", "5:B;9:<empty>", None, None)
    assert c.start_episode() == "A"
    seq = []
    for f in range(12):
        c.step(f, {})
        seq.append(c.current)
    assert seq == ["A"] * 5 + ["B"] * 4 + [""] * 3, seq
    assert c.start_episode() == "A" and c.step(0, {}) is None

    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "p.txt"
        f.write_text("# comment\nP1\nP2\n\nP3\n")
        c = PromptController.from_args("A", None, str(f), str(f))
        assert [c.start_episode() for _ in range(4)] == ["P1", "P2", "P3", "P1"]
        c.retry_episode()
        assert c.start_episode() == "P1"
        sw = c.step(3, {"prompt_hotkey": 2})
        assert sw.new == "P2" and sw.source == "hotkey" and c.current == "P2"
        assert c.step(4, {"prompt_hotkey": 9}) is None
    assert PromptController.from_args("A", None, None, None) is None
    print("ok  controller")


def test_runner():
    lat_frames = int(np.ceil(LATENCY * FPS))
    for flush in (False, True):
        for sf in (60, 0, 43):          # 43: lands around a normal chunk hand-over
            served, calls = rollout(flush, sf)
            first_b = served.index("B")
            assert set(served[:sf]) <= {"A"}, "new prompt served before the switch"
            assert "A" not in served[first_b:], "old prompt came back after the switch"
            delay = first_b - sf
            if flush:
                assert delay == 0, f"flush: expected immediate, got {delay} frames"
            else:
                assert delay <= lat_frames + 2, f"no flush: switch took {delay} frames (> latency {lat_frames})"
            print(f"ok  runner flush={flush!s:5} switch@{sf:3d}: new prompt after {delay} frames "
                  f"(inference latency {lat_frames} frames), {len(calls)} chunks computed")


def test_dataset_per_frame_task():
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    with tempfile.TemporaryDirectory() as d:
        ds = LeRobotDataset.create("test/prompt_switch", fps=30, root=Path(d) / "ds", use_videos=False,
                                   features={"observation.state": {"dtype": "float32", "shape": (2,), "names": ["a", "b"]},
                                             "action": {"dtype": "float32", "shape": (2,), "names": ["a", "b"]}})
        for i in range(10):
            ds.add_frame({"observation.state": np.zeros(2, np.float32), "action": np.zeros(2, np.float32),
                          "task": "first prompt" if i < 6 else "second prompt"})
        ds.save_episode()
        ds.finalize() if hasattr(ds, "finalize") else None
        import pandas as pd
        root = Path(d) / "ds"
        frames = pd.concat(pd.read_parquet(f) for f in sorted(root.glob("data/*/*.parquet")))
        tasks = pd.read_parquet(root / "meta" / "tasks.parquet")
        names = dict(zip(tasks["task_index"], tasks.index))
        per_frame = [names[i] for i in frames["task_index"]]
        assert per_frame == ["first prompt"] * 6 + ["second prompt"] * 4, per_frame
    print("ok  dataset stores a per-frame task:", per_frame[4:8])


if __name__ == "__main__":
    test_controller()
    test_runner()
    test_dataset_per_frame_task()
    print("all passed")
