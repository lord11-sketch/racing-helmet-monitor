#!/usr/bin/env python3
"""Summarise and plot a serial log captured from the helmet monitor.

Capture a log with, for example:

    pio device monitor --quiet > session.csv

then:

    python tools/plot_log.py session.csv            # summary + plot window
    python tools/plot_log.py session.csv -o out.png # summary + image file
    python tools/plot_log.py session.csv --no-plot  # summary only

Lines starting with '#' are status messages from the firmware and are skipped,
as is anything else that is not a complete CSV row. Only the standard library
is needed for the summary; the plot needs matplotlib.
"""

import argparse
import csv
import sys

# Thresholds mirrored from src/config.h, drawn as reference lines only.
G_THRESHOLDS = ((2.5, "warn"), (4.0, "danger"), (6.0, "critical"))
HR_LIMITS = (40, 180)
ALERT_NAMES = ("OK", "WARN", "DNGR", "CRIT")

# Chart colours: two series at most per panel, colour-blind-safe pair.
SERIES_1 = "#2a78d6"
SERIES_2 = "#eb6834"
INK_MUTED = "#52514e"
SURFACE = "#fcfcfb"


def read_log(path):
    """Return (column_names, rows) where rows is a list of {name: float}."""
    header = None
    rows = []
    with open(path, newline="", encoding="utf-8", errors="replace") as handle:
        for fields in csv.reader(handle):
            if not fields or fields[0].lstrip().startswith("#"):
                continue
            fields = [f.strip() for f in fields]
            if fields[0] == "time_ms":
                header = fields
                continue
            if header is None or len(fields) != len(header):
                continue  # boot noise or a truncated line
            try:
                rows.append({name: float(value) for name, value in zip(header, fields)})
            except ValueError:
                continue
    if header is None:
        raise SystemExit(f"{path}: no CSV header line (starting 'time_ms,') found")
    return header, rows


def gsr_column(header):
    return "gsr_mv" if "gsr_mv" in header else "gsr_raw"


def summarise(header, rows):
    t = [r["time_ms"] for r in rows]
    duration = (t[-1] - t[0]) / 1000.0
    print(f"rows: {len(rows)}   duration: {duration:.1f} s")
    if len(rows) > 1 and duration > 0:
        print(f"mean log rate: {(len(rows) - 1) / duration:.1f} Hz")

    def stats(name, label, valid=lambda v: True):
        values = [r[name] for r in rows if name in r and valid(r[name])]
        if values:
            mean = sum(values) / len(values)
            print(f"{label:<22} min {min(values):8.2f}  mean {mean:8.2f}  max {max(values):8.2f}  (n={len(values)})")
        else:
            print(f"{label:<22} no valid readings")

    stats("g_total", "g total (g)")
    stats("g_peak_window", "g peak per row (g)")
    stats("hr_bpm", "heart rate (bpm)", lambda v: v > 0)
    stats("spo2_pct", "SpO2 (%)", lambda v: v > 0)
    stats("gsr_index", "GSR arousal index")
    stats(gsr_column(header), "GSR reading")

    for level in (1, 2, 3):
        count = sum(1 for r in rows if int(r["alert"]) == level)
        print(f"rows at alert {ALERT_NAMES[level]:<5}: {count}")
    if "status" in header:
        clipped = sum(1 for r in rows if int(r["status"]) & 0x10)
        print(f"rows with accelerometer clipping: {clipped}")


def plot(header, rows, output):
    try:
        import matplotlib
        if output:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping the plot (pip install matplotlib)", file=sys.stderr)
        return

    t = [(r["time_ms"] - rows[0]["time_ms"]) / 1000.0 for r in rows]

    def column(name, blank_zero=False):
        return [float("nan") if (blank_zero and r[name] <= 0) else r[name] for r in rows]

    # One measure per panel on a shared time axis; never two y-scales in one panel.
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(11, 10), facecolor=SURFACE,
                             gridspec_kw={"height_ratios": [3, 3, 2, 2, 1.4]})
    for ax in axes:
        ax.set_facecolor(SURFACE)
        ax.grid(True, color="#e4e3df", linewidth=0.6)
        ax.tick_params(colors=INK_MUTED, labelsize=9)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color("#c9c8c2")

    ax = axes[0]
    ax.plot(t, column("g_peak_window"), color=SERIES_2, linewidth=1.2, label="peak within each log row")
    ax.plot(t, column("g_total"), color=SERIES_1, linewidth=1.6, label="latest sample")
    for value, name in G_THRESHOLDS:
        ax.axhline(value, color=INK_MUTED, linewidth=0.8, linestyle="--")
        ax.annotate(f"{name} {value:g} g", xy=(1.0, value), xycoords=("axes fraction", "data"),
                    xytext=(-4, 3), textcoords="offset points", ha="right", fontsize=8, color=INK_MUTED)
    ax.set_ylabel("dynamic g", color=INK_MUTED)
    ax.set_title("G-force (gravity removed)", loc="left", fontsize=11)
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, fontsize=8, frameon=False)

    ax = axes[1]
    ax.plot(t, column("hr_bpm", blank_zero=True), color=SERIES_1, linewidth=1.6, label="beat-to-beat (used for alerts)")
    if "hr_maxim_bpm" in header:
        ax.plot(t, column("hr_maxim_bpm", blank_zero=True), color=SERIES_2, linewidth=1.2,
                label="Maxim window algorithm (diagnostic)")
    for value in HR_LIMITS:
        ax.axhline(value, color=INK_MUTED, linewidth=0.8, linestyle="--")
    ax.set_ylabel("bpm", color=INK_MUTED)
    ax.set_title("Heart rate (gaps = no reading)", loc="left", fontsize=11)
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, fontsize=8, frameon=False)

    ax = axes[2]
    ax.plot(t, column("spo2_pct", blank_zero=True), color=SERIES_1, linewidth=1.6)
    ax.set_ylabel("%", color=INK_MUTED)
    ax.set_title("SpO2 (gaps = no reading)", loc="left", fontsize=11)

    ax = axes[3]
    ax.plot(t, column("gsr_index"), color=SERIES_1, linewidth=1.6)
    ax.set_ylim(-2, 102)
    ax.set_ylabel("0-100", color=INK_MUTED)
    ax.set_title("GSR arousal index (relative to calibration baseline)", loc="left", fontsize=11)

    ax = axes[4]
    ax.step(t, column("alert"), where="post", color=SERIES_1, linewidth=1.6)
    ax.set_yticks(range(4))
    ax.set_yticklabels(ALERT_NAMES)
    ax.set_ylim(-0.3, 3.3)
    ax.set_title("Alert level", loc="left", fontsize=11)
    ax.set_xlabel("time since first row (s)", color=INK_MUTED)

    fig.tight_layout()
    if output:
        fig.savefig(output, dpi=150, facecolor=SURFACE)
        print(f"wrote {output}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("logfile", help="text file captured from the serial port")
    parser.add_argument("-o", "--output", help="write the plot to this image file instead of opening a window")
    parser.add_argument("--no-plot", action="store_true", help="print the summary only")
    args = parser.parse_args()

    header, rows = read_log(args.logfile)
    if not rows:
        raise SystemExit(f"{args.logfile}: header found but no data rows")
    summarise(header, rows)
    if not args.no_plot:
        plot(header, rows, args.output)


if __name__ == "__main__":
    main()
