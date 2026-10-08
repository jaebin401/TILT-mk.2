"""Extract CSV / EVT lines from an idf.py monitor log of dcm_rocking_test and plot them.

Usage:
  python3 tools/dcm_log/extract.py MONITOR_LOG [--out PREFIX] [--no-plot]

Writes PREFIX.csv (one row per control cycle), PREFIX_events.txt and, when
matplotlib is installed, PREFIX.png with the DCM edge margins, swing phases,
roll and cycle time.
"""
import argparse
import csv
import os
import re
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*m")


def parse(path):
    header, rows, events = None, [], []
    with open(path, errors="replace") as fh:
        for raw in fh:
            line = ANSI.sub("", raw).strip()
            idx = line.find("CSV,")
            if idx >= 0:
                fields = line[idx + 4:].split(",")
                if fields and fields[0] == "t_ms":
                    header = fields
                elif header and len(fields) == len(header):
                    rows.append(fields)
                continue
            if "EVT " in line or "== run:" in line or "STOP" in line:
                events.append(line)
    return header, rows, events


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log")
    parser.add_argument("--out")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    prefix = args.out or os.path.splitext(args.log)[0]
    header, rows, events = parse(args.log)
    if not header:
        sys.exit("no CSV header found (press 'c' in the monitor before the run)")
    with open(prefix + ".csv", "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)
    with open(prefix + "_events.txt", "w") as fh:
        fh.write("\n".join(events) + "\n")
    print(f"{len(rows)} rows -> {prefix}.csv, {len(events)} events -> {prefix}_events.txt")
    if args.no_plot or not rows:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot")
        return

    col = {name: i for i, name in enumerate(header)}

    def series(name):
        out = []
        for r in rows:
            try:
                out.append(float(r[col[name]]))
            except ValueError:
                out.append(float("nan"))
        return out

    t = [(v - series("t_ms")[0]) / 1000.0 for v in series("t_ms")]
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for i, r in enumerate(rows):
        if r[col["phase"]] == "SWING":
            colour = "#cfe3ff" if r[col["swing"]] == "R" else "#ffe0c2"
            for a in ax:
                a.axvspan(t[i], t[min(i + 1, len(t) - 1)], color=colour, lw=0)
    ax[0].plot(t, series("inner_L"), label="DCM past LEFT inner edge")
    ax[0].plot(t, series("inner_R"), label="DCM past RIGHT inner edge")
    ax[0].axhline(0, color="k", lw=0.8)
    ax[0].set_ylabel("mm")
    ax[0].legend(loc="upper right")
    ax[0].set_title("blue = right swing, orange = left swing")
    ax[1].plot(t, series("dcm_y"), label="dcm_y")
    ax[1].plot(t, series("com_y"), label="com_y")
    ax[1].plot(t, series("inside_swing"), label="DCM inside swing foot")
    ax[1].set_ylabel("mm (stance-foot frame)")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, series("roll"), label="roll")
    ax[2].plot(t, [v / 10 for v in series("gx")], label="gyro x / 10")
    ax[2].set_ylabel("deg, deg/s")
    ax[2].legend(loc="upper right")
    ax[3].plot(t, series("cycle_us"), label="cycle")
    ax[3].plot(t, series("readback_us"), label="read-back")
    ax[3].axhline(10000, color="r", lw=0.8)
    ax[3].set_ylabel("us")
    ax[3].set_xlabel("s")
    ax[3].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(prefix + ".png", dpi=120)
    print(f"plot -> {prefix}.png")


if __name__ == "__main__":
    main()
