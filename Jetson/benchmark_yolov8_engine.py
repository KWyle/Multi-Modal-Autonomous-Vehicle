"""
╔══════════════════════════════════════════════════════════════════════════════╗
║       YOLOv8n TensorRT Engine Benchmarker — Jetson Orin Nano ADAS          ║
║                                                                              ║
║  Metrics:                                                                    ║
║    • Latency  — mean, std, P50 / P95 / P99                                  ║
║    • Throughput — FPS (sustained & burst)                                    ║
║    • Accuracy  — mAP@0.5, mAP@0.5:0.95, Precision, Recall, F1              ║
║    • ADAS Safety — FPR, FNR, Miss Rate per class                            ║
║    • Memory    — GPU VRAM, system RAM peak                                   ║
║    • Power     — avg / peak watts  (jtop or tegrastats)                     ║
║    • Thermal   — CPU / GPU temperature                                       ║
║    • Stability — latency variance & jitter over time                         ║
╚══════════════════════════════════════════════════════════════════════════════╝

Usage
-----
  # Latency + throughput only (no dataset required):
  python benchmark_yolov8_engine.py --model yolov8n.engine

  # Full accuracy + ADAS safety metrics (requires COCO-format dataset):
  python benchmark_yolov8_engine.py --model yolov8n.engine \
      --data path/to/data.yaml --split val

  # Custom image size & warm-up:
  python benchmark_yolov8_engine.py --model yolov8n.engine \
      --imgsz 640 --warmup 50 --runs 500

  # Save results:
  python benchmark_yolov8_engine.py --model yolov8n.engine \
      --output results/benchmark.json

  # Save plots as PNGs:
  python benchmark_yolov8_engine.py --model yolov8n.engine \
      --plots --plots-dir results/plots

Dependencies
------------
  pip install ultralytics psutil                  # core
  pip install jtop                                 # Jetson power/thermal (optional)
  # tegrastats is pre-installed on Jetson OS
  pip install matplotlib                             # optional plotting
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import warnings
from collections import defaultdict
from pathlib import Path

import numpy as np
import psutil

warnings.filterwarnings("ignore")

# ─── Optional imports ────────────────────────────────────────────────────────

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False
    print("[WARN] ultralytics not found. Install with: pip install ultralytics")

try:
    from jtop import jtop as JTop
    HAS_JTOP = True
except ImportError:
    HAS_JTOP = False

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

try:
    import matplotlib
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
    matplotlib.use('Agg')
except ImportError:
    HAS_MATPLOTLIB = False
    plt = None


# ═══════════════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def get_gpu_memory_mb() -> float:
    """Return current GPU memory usage in MB (nvidia-smi or torch fallback)."""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            timeout=3,
        ).decode().strip()
        return float(out.split("\n")[0])
    except Exception:
        pass
    if HAS_TORCH and torch.cuda.is_available():
        return torch.cuda.memory_allocated() / 1024 ** 2
    return 0.0


def get_ram_usage_mb() -> float:
    return psutil.virtual_memory().used / 1024 ** 2


def percentile(data: list, p: float) -> float:
    arr = sorted(data)
    idx = (len(arr) - 1) * p / 100
    lo, hi = int(idx), min(int(idx) + 1, len(arr) - 1)
    return arr[lo] + (arr[hi] - arr[lo]) * (idx - lo)


def color(text: str, code: int) -> str:
    return f"\033[{code}m{text}\033[0m"

def green(t):  return color(t, 32)
def yellow(t): return color(t, 33)
def cyan(t):   return color(t, 36)
def bold(t):   return color(t, 1)
def red(t):    return color(t, 31)


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def plot_latency_series(latencies_ms: list[float], out_dir: Path) -> None:
    if not HAS_MATPLOTLIB or not latencies_ms:
        return
    ensure_dir(out_dir)
    arr = np.array(latencies_ms, dtype=float)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(arr, alpha=0.7, label="Latency per pass")
    if len(arr) > 10:
        window = min(50, len(arr) // 4)
        smooth = np.convolve(arr, np.ones(window) / window, mode="valid")
        ax.plot(range(window - 1, len(arr)), smooth, label=f"{window}-step avg", color="#ff7f0e")
    ax.set_title("Inference latency over time")
    ax.set_xlabel("Pass")
    ax.set_ylabel("Latency (ms)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "latency_time_series.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(arr, bins=30, color="#4c72b0", edgecolor="black", alpha=0.8)
    ax.set_title("Latency distribution")
    ax.set_xlabel("Latency (ms)")
    ax.set_ylabel("Frames")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "latency_histogram.png", dpi=150)
    plt.close(fig)


def plot_accuracy_summary(results: dict, out_dir: Path) -> None:
    if not HAS_MATPLOTLIB or "accuracy" not in results:
        return
    ensure_dir(out_dir)
    acc = results["accuracy"]
    labels = ["mAP@0.5", "mAP@0.5:0.95", "Precision", "Recall", "F1"]
    values = [acc.get("map50", 0), acc.get("map50_95", 0), acc.get("precision", 0), acc.get("recall", 0), acc.get("f1", 0)]

    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(labels, values, color=["#2ca02c", "#1f77b4", "#ff7f0e", "#9467bd", "#8c564b"], alpha=0.85)
    ax.set_ylim(0, 1)
    ax.set_title("Accuracy metrics")
    ax.set_ylabel("Score")
    ax.grid(axis="y", alpha=0.3)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, value + 0.02, f"{value:.3f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / "accuracy_metrics.png", dpi=150)
    plt.close(fig)

    latency = results.get("latency", {}).get("latency_ms", {})
    if latency:
        fig, ax1 = plt.subplots(figsize=(10, 5))
        x = np.arange(2)
        ax2 = ax1.twinx()
        ax1.bar(x, [acc.get("map50", 0), acc.get("map50_95", 0)], width=0.4,
                color="#1f77b4", alpha=0.7, label="mAP")
        ax1.set_xticks(x)
        ax1.set_xticklabels(["mAP@0.5", "mAP@0.5:0.95"])
        ax1.set_ylabel("Accuracy")
        ax1.set_ylim(0, 1)

        lat_values = [latency.get("mean", 0), latency.get("p95", 0)]
        ax2.plot(x, lat_values, color="#d62728", marker="o", linewidth=2, label="Latency (ms)")
        ax2.set_ylabel("Latency (ms)")
        ax2.set_ylim(0, max(lat_values) * 1.2 if max(lat_values) > 0 else 1)

        ax1.set_title("Accuracy vs. latency")
        ax1.grid(True, alpha=0.2)
        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(out_dir / "accuracy_vs_latency.png", dpi=150)
        plt.close(fig)


def plot_per_class_metrics(results: dict, out_dir: Path) -> None:
    if not HAS_MATPLOTLIB or "accuracy" not in results:
        return
    ensure_dir(out_dir)
    per_class = results["accuracy"].get("per_class", {})
    if not per_class:
        return
    labels = []
    precision = []
    recall = []
    f1 = []
    for name, v in sorted(per_class.items(), key=lambda item: item[0])[:12]:
        labels.append(name)
        precision.append(v.get("precision", 0))
        recall.append(v.get("recall", 0))
        f1.append(v.get("f1", 0))
    if not labels:
        return

    x = np.arange(len(labels))
    width = 0.25
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.bar(x - width, precision, width, label="Precision", color="#2ca02c")
    ax.bar(x, recall, width, label="Recall", color="#1f77b4")
    ax.bar(x + width, f1, width, label="F1", color="#ff7f0e")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylim(0, 1)
    ax.set_title("Per-class accuracy metrics")
    ax.set_ylabel("Score")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "per_class_metrics.png", dpi=150)
    plt.close(fig)


def generate_plots(results: dict, plots_dir: str) -> None:
    if not HAS_MATPLOTLIB:
        print("[WARN] matplotlib not installed. Install with: pip install matplotlib")
        return
    out_dir = Path(plots_dir)
    print(f"\n  Generating plots in: {out_dir}")
    raw_latencies = results.get("_raw_latencies_ms", [])
    plot_latency_series(raw_latencies, out_dir)
    plot_accuracy_summary(results, out_dir)
    plot_per_class_metrics(results, out_dir)
# ═══════════════════════════════════════════════════════════════════════════════

class PowerMonitor:
    """
    Background thread that samples Jetson power and temperature.
    Tries jtop first; falls back to tegrastats; falls back to nvidia-smi.
    """

    def __init__(self, interval_s: float = 0.5):
        self.interval = interval_s
        self._samples_power: list[float] = []
        self._samples_cpu_temp: list[float] = []
        self._samples_gpu_temp: list[float] = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._backend = self._detect_backend()

    def _detect_backend(self) -> str:
        if HAS_JTOP:
            return "jtop"
        try:
            subprocess.check_output(["which", "tegrastats"], timeout=2)
            return "tegrastats"
        except Exception:
            pass
        try:
            subprocess.check_output(["nvidia-smi", "--query-gpu=temperature.gpu",
                                     "--format=csv,noheader,nounits"], timeout=2)
            return "nvidia-smi"
        except Exception:
            pass
        return "none"

    # ── jtop sampling ──────────────────────────────────────────────────────
    def _run_jtop(self):
        try:
            with JTop(self.interval) as jetson:
                while self._running:
                    # Total board power (mW → W)
                    total_w = jetson.power.get("tot", {}).get("power", 0) / 1000.0
                    self._samples_power.append(total_w)
                    # Temperatures
                    temps = jetson.temperature
                    self._samples_cpu_temp.append(temps.get("CPU", 0.0))
                    self._samples_gpu_temp.append(temps.get("GPU", 0.0))
        except Exception as e:
            print(f"[WARN] jtop error: {e}")

    # ── tegrastats sampling ────────────────────────────────────────────────
    def _run_tegrastats(self):
        proc = subprocess.Popen(
            ["tegrastats", f"--interval", str(int(self.interval * 1000))],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True,
        )
        try:
            for line in proc.stdout:
                if not self._running:
                    break
                # Parse total power: "VDD_IN 5000mW/6000mW" → 5.0 W
                for token in line.split():
                    if "mW" in token and "/" in token:
                        try:
                            mw = float(token.split("/")[0].replace("mW", ""))
                            self._samples_power.append(mw / 1000.0)
                            break
                        except ValueError:
                            pass
                # Parse CPU temperature: "CPU@45.5C"
                for token in line.split():
                    if token.startswith("CPU@") and "C" in token:
                        try:
                            self._samples_cpu_temp.append(
                                float(token.replace("CPU@", "").replace("C", ""))
                            )
                        except ValueError:
                            pass
                    if token.startswith("GPU@") and "C" in token:
                        try:
                            self._samples_gpu_temp.append(
                                float(token.replace("GPU@", "").replace("C", ""))
                            )
                        except ValueError:
                            pass
        finally:
            proc.terminate()

    # ── nvidia-smi fallback ────────────────────────────────────────────────
    def _run_nvidia_smi(self):
        while self._running:
            try:
                out = subprocess.check_output(
                    ["nvidia-smi", "--query-gpu=temperature.gpu,power.draw",
                     "--format=csv,noheader,nounits"],
                    timeout=2,
                ).decode().strip()
                parts = out.split(",")
                if len(parts) >= 2:
                    self._samples_gpu_temp.append(float(parts[0].strip()))
                    self._samples_power.append(float(parts[1].strip()))
            except Exception:
                pass
            time.sleep(self.interval)

    def start(self):
        self._running = True
        fn = {"jtop": self._run_jtop,
              "tegrastats": self._run_tegrastats,
              "nvidia-smi": self._run_nvidia_smi,
              "none": None}[self._backend]
        if fn is None:
            return
        self._thread = threading.Thread(target=fn, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def summary(self) -> dict:
        def stats(lst):
            if not lst:
                return {"mean": None, "peak": None, "unit": None}
            a = np.array(lst, dtype=float)
            return {"mean": round(float(a.mean()), 2), "peak": round(float(a.max()), 2)}

        return {
            "backend": self._backend,
            "power_W": stats(self._samples_power),
            "cpu_temp_C": stats(self._samples_cpu_temp),
            "gpu_temp_C": stats(self._samples_gpu_temp),
        }


# ═══════════════════════════════════════════════════════════════════════════════
#  Latency & Throughput Benchmark
# ═══════════════════════════════════════════════════════════════════════════════

class LatencyBenchmark:
    """
    Runs N inference passes and captures per-pass timings.
    Supports dummy (random) frames or a list of image paths.
    """

    def __init__(self, model, imgsz: int = 640, device: str = "0"):
        self.model = model
        self.imgsz = imgsz
        self.device = device

    def run(self, n_warmup: int = 50, n_runs: int = 500,
            image_paths: list | None = None) -> dict:

        dummy = image_paths is None
        if dummy:
            # Pure random tensor — no I/O, pure engine speed
            import numpy as np
            frames = [np.random.randint(0, 255, (self.imgsz, self.imgsz, 3),
                                        dtype=np.uint8)] * max(n_warmup, 1)
        else:
            frames = image_paths * (n_warmup // max(len(image_paths), 1) + 1)

        print(f"\n  Warm-up ({n_warmup} passes)...", end="", flush=True)
        for frame in frames[:n_warmup]:
            self.model.predict(frame, imgsz=self.imgsz, device=self.device,
                               verbose=False, conf=0.25)
        print(" done.")

        print(f"  Measuring ({n_runs} passes)...", end="", flush=True)
        latencies_ms: list[float] = []
        mem_gpu_before = get_gpu_memory_mb()

        t_total_start = time.perf_counter()
        for i in range(n_runs):
            src = frames[i % len(frames)] if not dummy else frames[0]
            t0 = time.perf_counter()
            self.model.predict(src, imgsz=self.imgsz, device=self.device,
                               verbose=False, conf=0.25)
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000)
        t_total = time.perf_counter() - t_total_start

        mem_gpu_after = get_gpu_memory_mb()
        print(" done.")

        arr = np.array(latencies_ms)
        return {
            "n_runs": n_runs,
            "n_warmup": n_warmup,
            "latency_ms": {
                "mean":  round(float(arr.mean()), 3),
                "std":   round(float(arr.std()), 3),
                "min":   round(float(arr.min()), 3),
                "max":   round(float(arr.max()), 3),
                "p50":   round(percentile(latencies_ms, 50), 3),
                "p95":   round(percentile(latencies_ms, 95), 3),
                "p99":   round(percentile(latencies_ms, 99), 3),
                "jitter_ms": round(float(arr.std()), 3),   # 1-sigma jitter
            },
            "throughput": {
                "fps_mean":       round(1000.0 / float(arr.mean()), 2),
                "fps_sustained":  round(n_runs / t_total, 2),
                "fps_p95":        round(1000.0 / percentile(latencies_ms, 95), 2),
            },
            "memory_gpu_MB": {
                "before_warmup": round(mem_gpu_before, 1),
                "after_benchmark": round(mem_gpu_after, 1),
                "delta": round(mem_gpu_after - mem_gpu_before, 1),
            },
            "_raw_latencies_ms": latencies_ms,   # kept for stability plot
        }


# ═══════════════════════════════════════════════════════════════════════════════
#  Accuracy & ADAS Safety Benchmark
# ═══════════════════════════════════════════════════════════════════════════════

class AccuracyBenchmark:
    """
    Runs model.val() on a COCO-format dataset and extracts:
      • mAP@0.5, mAP@0.5:0.95
      • Per-class precision, recall, F1
      • ADAS safety: FPR, FNR, miss-rate per class
    """

    # ADAS-critical classes (COCO labels).  Tune to your deployment classes.
    ADAS_CRITICAL = {
        "person", "bicycle", "car", "motorcycle", "bus", "truck",
        "traffic light", "stop sign",
    }

    def __init__(self, model, data_yaml: str, split: str = "val",
                 imgsz: int = 640, device: str = "0"):
        self.model = model
        self.data_yaml = data_yaml
        self.split = split
        self.imgsz = imgsz
        self.device = device

    def run(self) -> dict:
        print(f"\n  Running validation on '{self.data_yaml}' ({self.split} split)...")

        results = self.model.val(
            data=self.data_yaml,
            split=self.split,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

        # ── Core mAP metrics ─────────────────────────────────────────────
        metrics = results.results_dict
        map50    = round(float(metrics.get("metrics/mAP50(B)",   0)), 4)
        map5095  = round(float(metrics.get("metrics/mAP50-95(B)", 0)), 4)
        prec     = round(float(metrics.get("metrics/precision(B)", 0)), 4)
        rec      = round(float(metrics.get("metrics/recall(B)",    0)), 4)
        f1       = round(2 * prec * rec / max(prec + rec, 1e-9), 4)

        # ── Per-class breakdown ───────────────────────────────────────────
        per_class: dict[str, dict] = {}
        names = results.names  # {0: 'person', 1: 'bicycle', ...}

        if hasattr(results, "box") and results.box is not None:
            box = results.box
            for i, name in names.items():
                try:
                    p  = float(box.p[i])
                    r  = float(box.r[i])
                    ap = float(box.ap50[i])
                    f  = 2 * p * r / max(p + r, 1e-9)
                    # ADAS safety metrics
                    # FNR = 1 - Recall (missed detections)
                    # FPR ≈ 1 - Precision (false alarms)
                    fnr = round(1.0 - r, 4)
                    fpr = round(1.0 - p, 4)
                    per_class[name] = {
                        "precision": round(p, 4),
                        "recall":    round(r, 4),
                        "f1":        round(f, 4),
                        "ap50":      round(ap, 4),
                        "fnr":       fnr,    # miss rate
                        "fpr":       fpr,    # false alarm rate
                        "adas_critical": name in self.ADAS_CRITICAL,
                    }
                except (IndexError, AttributeError):
                    pass

        # ── ADAS summary ──────────────────────────────────────────────────
        critical = {k: v for k, v in per_class.items() if v.get("adas_critical")}
        adas_summary = {}
        if critical:
            avg_fnr = np.mean([v["fnr"] for v in critical.values()])
            avg_fpr = np.mean([v["fpr"] for v in critical.values()])
            worst_fnr_cls = max(critical, key=lambda k: critical[k]["fnr"])
            worst_fpr_cls = max(critical, key=lambda k: critical[k]["fpr"])
            adas_summary = {
                "critical_classes_evaluated": list(critical.keys()),
                "avg_miss_rate_FNR": round(float(avg_fnr), 4),
                "avg_false_alarm_rate_FPR": round(float(avg_fpr), 4),
                "worst_miss_rate_class":  worst_fnr_cls,
                "worst_false_alarm_class": worst_fpr_cls,
                "safety_note": (
                    "FNR (miss rate) is the primary safety hazard — a missed "
                    "pedestrian or vehicle is a collision risk. FPR (false alarm) "
                    "degrades driver trust and causes alarm fatigue."
                ),
            }

        return {
            "map50":     map50,
            "map50_95":  map5095,
            "precision": prec,
            "recall":    rec,
            "f1":        f1,
            "per_class": per_class,
            "adas_safety": adas_summary,
        }


# ═══════════════════════════════════════════════════════════════════════════════
#  Stability Analysis
# ═══════════════════════════════════════════════════════════════════════════════

def analyze_stability(latencies_ms: list[float], window: int = 50) -> dict:
    """
    Compute rolling statistics to detect thermal throttling or memory pressure
    over time — common on edge devices during sustained inference.
    """
    arr = np.array(latencies_ms)
    n = len(arr)

    rolling_mean, rolling_std = [], []
    for i in range(0, n - window + 1, window):
        chunk = arr[i: i + window]
        rolling_mean.append(round(float(chunk.mean()), 3))
        rolling_std.append(round(float(chunk.std()), 3))

    if len(rolling_mean) < 2:
        return {"windows": rolling_mean, "drift_detected": False}

    # Drift = last window mean vs first window mean > 10%
    drift_pct = (rolling_mean[-1] - rolling_mean[0]) / max(rolling_mean[0], 1e-9) * 100
    drift_detected = abs(drift_pct) > 10.0

    return {
        "window_size": window,
        "rolling_mean_ms": rolling_mean,
        "rolling_std_ms": rolling_std,
        "latency_drift_pct": round(drift_pct, 2),
        "drift_detected": drift_detected,
        "drift_note": (
            "Latency increased >10% over the benchmark run — likely thermal "
            "throttling. Check GPU temperature and cooling."
            if drift_detected else "No significant latency drift detected."
        ),
    }


# ═══════════════════════════════════════════════════════════════════════════════
#  Report Printer
# ═══════════════════════════════════════════════════════════════════════════════

def print_report(results: dict):
    W = 68
    def hdr(t): print(f"\n{bold(cyan('━' * 3 + f'  {t}  ' + '━' * (W - len(t) - 6)))}")
    def row(k, v, warn=False, good=False):
        col = yellow if warn else (green if good else str)
        print(f"    {k:<35} {col(str(v))}")

    print()
    print(bold(f"  {'YOLOv8n TensorRT Engine — Benchmark Report':^{W}}"))
    print(bold(f"  {'=' * W}"))

    # ── Model info ────────────────────────────────────────────────────────
    hdr("Model & Config")
    row("Model",      results.get("model_path"))
    row("Image size", results.get("imgsz"))
    row("Device",     results.get("device"))
    row("Timestamp",  results.get("timestamp"))

    # ── Latency ───────────────────────────────────────────────────────────
    lat = results.get("latency", {})
    if lat:
        hdr("Latency")
        ms = lat.get("latency_ms", {})
        row("Mean latency",        f"{ms.get('mean')} ms")
        row("Std dev (jitter)",    f"{ms.get('std')} ms",
            warn=(ms.get("std", 0) > ms.get("mean", 1) * 0.1))
        row("P50 (median)",        f"{ms.get('p50')} ms")
        row("P95",                 f"{ms.get('p95')} ms",
            warn=(ms.get("p95", 0) > 66))   # >66ms → <15 FPS worst case
        row("P99",                 f"{ms.get('p99')} ms",
            warn=(ms.get("p99", 0) > 100))
        row("Min / Max",           f"{ms.get('min')} / {ms.get('max')} ms")

        th = lat.get("throughput", {})
        row("FPS (mean)",          f"{th.get('fps_mean')}",
            good=(th.get("fps_mean", 0) >= 30))
        row("FPS (sustained)",     f"{th.get('fps_sustained')}")
        row("FPS at P95 latency",  f"{th.get('fps_p95')}")

    # ── Memory ────────────────────────────────────────────────────────────
    mem = lat.get("memory_gpu_MB", {}) if lat else {}
    if mem:
        hdr("GPU Memory")
        row("Before warmup",       f"{mem.get('before_warmup')} MB")
        row("After benchmark",     f"{mem.get('after_benchmark')} MB")
        row("Delta (engine load)", f"{mem.get('delta')} MB",
            warn=(mem.get("delta", 0) > 200))

    # ── Power & Thermal ───────────────────────────────────────────────────
    pwr = results.get("power_thermal", {})
    if pwr:
        hdr("Power & Thermal")
        row("Monitor backend", pwr.get("backend"))
        p = pwr.get("power_W", {})
        row("Avg power",  f"{p.get('mean')} W" if p.get("mean") else "N/A",
            warn=(p.get("mean") or 0) > 10)
        row("Peak power", f"{p.get('peak')} W" if p.get("peak") else "N/A")
        ct = pwr.get("cpu_temp_C", {})
        row("CPU temp (avg/peak)",
            f"{ct.get('mean')} / {ct.get('peak')} °C" if ct.get("mean") else "N/A",
            warn=(ct.get("peak") or 0) > 75)
        gt = pwr.get("gpu_temp_C", {})
        row("GPU temp (avg/peak)",
            f"{gt.get('mean')} / {gt.get('peak')} °C" if gt.get("mean") else "N/A",
            warn=(gt.get("peak") or 0) > 75)

    # ── Stability ─────────────────────────────────────────────────────────
    stab = results.get("stability", {})
    if stab:
        hdr("Stability")
        drift = stab.get("latency_drift_pct")
        row("Latency drift (first→last window)",
            f"{drift:+.2f}%" if drift is not None else "N/A",
            warn=stab.get("drift_detected", False))
        row("Drift note", stab.get("drift_note", ""))

    # ── Accuracy ──────────────────────────────────────────────────────────
    acc = results.get("accuracy", {})
    if acc:
        hdr("Accuracy")
        row("mAP@0.5",      acc.get("map50"),    good=(acc.get("map50", 0) >= 0.50))
        row("mAP@0.5:0.95", acc.get("map50_95"), good=(acc.get("map50_95", 0) >= 0.30))
        row("Precision",    acc.get("precision"))
        row("Recall",       acc.get("recall"))
        row("F1",           acc.get("f1"))

        adas = acc.get("adas_safety", {})
        if adas:
            hdr("ADAS Safety (Critical Classes)")
            row("Avg miss rate (FNR)",
                f"{adas.get('avg_miss_rate_FNR'):.1%}",
                warn=(adas.get("avg_miss_rate_FNR", 0) > 0.10))
            row("Avg false alarm rate (FPR)",
                f"{adas.get('avg_false_alarm_rate_FPR'):.1%}",
                warn=(adas.get("avg_false_alarm_rate_FPR", 0) > 0.20))
            row("Worst miss-rate class",  adas.get("worst_miss_rate_class"),
                warn=True)
            row("Worst false-alarm class", adas.get("worst_false_alarm_class"))
            print(f"\n    {yellow('⚠')}  {adas.get('safety_note','')}")

        pc = acc.get("per_class", {})
        if pc:
            hdr("Per-Class Breakdown")
            print(f"    {'Class':<20} {'P':>6} {'R':>6} {'F1':>6} "
                  f"{'AP50':>7} {'FNR':>7} {'FPR':>7}  ADAS")
            print(f"    {'─'*20} {'─'*6} {'─'*6} {'─'*6} {'─'*7} {'─'*7} {'─'*7}  ────")
            for name, v in sorted(pc.items()):
                crit = "🚨 CRIT" if v.get("adas_critical") else ""
                fnr_col = red if v["fnr"] > 0.15 else (yellow if v["fnr"] > 0.10 else green)
                fnr_str = fnr_col("{:>7.3f}".format(v["fnr"]))
                print(
                    "    {:<20} {:>6.3f} {:>6.3f} {:>6.3f} {:>7.3f} {} {:>7.3f}  {}".format(
                        name, v["precision"], v["recall"], v["f1"],
                        v["ap50"], fnr_str, v["fpr"], crit
                    )
                )

    print(f"\n  {'═' * W}")
    print(f"  {green('✓')} Benchmark complete.\n")


# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description="YOLOv8n TensorRT Engine Benchmarker — Jetson Orin Nano ADAS",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--model",   required=True, help="Path to .engine model file")
    p.add_argument("--data",    default=None,  help="Path to data.yaml (for accuracy eval)")
    p.add_argument("--split",   default="val", choices=["val", "test"],
                   help="Dataset split to evaluate")
    p.add_argument("--imgsz",   type=int, default=640, help="Inference image size")
    p.add_argument("--device",  default="0",  help="CUDA device id")
    p.add_argument("--warmup",  type=int, default=50,  help="Warm-up passes")
    p.add_argument("--runs",    type=int, default=500, help="Timed inference passes")
    p.add_argument("--output",  default=None,
                   help="Save JSON results to this path (e.g. results/bench.json)")
    p.add_argument("--images",  default=None,
                   help="Directory of images for latency test (uses random frames if omitted)")
    p.add_argument("--plots", action="store_true",
                   help="Generate benchmark plots for latency and accuracy")
    p.add_argument("--plots-dir", default="results/plots",
                   help="Folder to save benchmark plots")
    return p.parse_args()


def collect_image_paths(directory: str | None) -> list | None:
    if directory is None:
        return None
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    paths = [str(p) for p in Path(directory).rglob("*") if p.suffix.lower() in exts]
    if not paths:
        print(f"[WARN] No images found in {directory} — using random frames.")
        return None
    return paths


def main():
    args = parse_args()

    if not HAS_ULTRALYTICS:
        sys.exit("ERROR: ultralytics is required. pip install ultralytics")

    if not Path(args.model).exists():
        sys.exit(f"ERROR: Model file not found: {args.model}")

    print(bold(cyan(f"\n  Loading model: {args.model}")))
    model = YOLO(args.model, task="detect")

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    results: dict = {
        "model_path": args.model,
        "imgsz":      args.imgsz,
        "device":     args.device,
        "timestamp":  timestamp,
    }

    # ── Power monitor ─────────────────────────────────────────────────────
    print(f"\n  Starting power / thermal monitor...")
    pw_monitor = PowerMonitor(interval_s=0.5)
    pw_monitor.start()

    # ── RAM usage snapshot ────────────────────────────────────────────────
    ram_before = get_ram_usage_mb()

    # ── Latency benchmark ─────────────────────────────────────────────────
    images = collect_image_paths(args.images)
    lat_bench = LatencyBenchmark(model, imgsz=args.imgsz, device=args.device)
    lat_results = lat_bench.run(n_warmup=args.warmup, n_runs=args.runs,
                                image_paths=images)
    results["latency"] = lat_results

    # ── Stability analysis ────────────────────────────────────────────────
    raw = lat_results.pop("_raw_latencies_ms", [])
    if raw:
        results["_raw_latencies_ms"] = raw
        results["stability"] = analyze_stability(raw)

    # ── RAM after benchmark ───────────────────────────────────────────────
    results["memory_ram_MB"] = {
        "before": round(ram_before, 1),
        "after":  round(get_ram_usage_mb(), 1),
        "delta":  round(get_ram_usage_mb() - ram_before, 1),
    }

    # ── Accuracy (optional) ───────────────────────────────────────────────
    if args.data:
        acc_bench = AccuracyBenchmark(model, data_yaml=args.data,
                                      split=args.split, imgsz=args.imgsz,
                                      device=args.device)
        results["accuracy"] = acc_bench.run()

    # ── Stop power monitor ────────────────────────────────────────────────
    pw_monitor.stop()
    results["power_thermal"] = pw_monitor.summary()

    # ── Print report ──────────────────────────────────────────────────────
    print_report(results)

    # ── Plot generation ────────────────────────────────────────────────────
    if args.plots:
        generate_plots(results, args.plots_dir)

    # ── Save JSON ─────────────────────────────────────────────────────────
    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(results, f, indent=2)
        print(f"  Results saved → {out_path}\n")

    return results


if __name__ == "__main__":
    main()