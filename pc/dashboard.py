#!/usr/bin/env python3
"""ShipGuard dashboard: stands in for the cloud + application layers.

Reads the CSV that pc/gateway.py writes (pc/data/latest.csv by default) and
shows, live:
  * a status strip: alert level, state, profile, ACK status, environment, and
    driver response times (alert -> person arrived -> ACK),
  * risk score with the L1/L2/L3 bands, ACK and person-arrival markers,
  * shock events (peak g), split into "during handling" (PIR active) and
    "unattended / vehicle" (PIR idle),
  * tilt angle against the profile's tilt limit,
  * temperature and humidity (two panels, never a dual axis).

Usage:
  python3 pc/dashboard.py                      # live, follows pc/data/latest.csv
  python3 pc/dashboard.py --csv pc/data/run_X.csv --static out.png   # report figure
"""
import argparse
import os

import matplotlib
import pandas as pd

# ---------------- palette (dataviz reference palette, light mode) ----------------
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
SERIES_1 = "#2a78d6"  # blue: risk, tilt, temperature, humidity, handling shocks
SERIES_2 = "#eb6834"  # orange: unattended shocks
STATUS = {0: "#0ca30c", 1: "#fab219", 2: "#ec835a", 3: "#d03b3b"}  # good/warning/serious/critical
LEVEL_TEXT = {0: "L0 OK", 1: "L1 WATCH", 2: "L2 CHECK", 3: "L3 INSPECT"}
LEVEL_ICON = {0: "●", 1: "▲", 2: "◆", 3: "■"}  # status colour never carries meaning alone
# Example profiles, mirrors firmware/shipguard/risk.cpp (not certified values).
PROFILE_TILT_LIMIT = {"GLASS": 30, "ELECTRONICS": 45, "ROBUST": 60}
PROFILE_G_CRIT = {"GLASS": 4.0, "ELECTRONICS": 8.0, "ROBUST": 14.0}
RISK_CAP = 200  # saturated shocks report 999; the axis is capped and labelled instead


def load(path: str) -> pd.DataFrame:
    try:
        d = pd.read_csv(path)
    except (FileNotFoundError, pd.errors.EmptyDataError):
        return pd.DataFrame()
    if d.empty:
        return d
    d["pc_time"] = pd.to_datetime(d["pc_time"])
    d["x"] = (d["pc_time"] - d["pc_time"].iloc[0]).dt.total_seconds()
    return d


def analyze(d: pd.DataFrame):
    """Alert episodes with driver response times, and shock attribution counts."""
    episodes = []
    cur = None
    prev_state = "MONITORING"
    for _, r in d.iterrows():
        if r["type"] == "ACK_LOG":
            if cur is not None and cur["ack"] is None:
                cur["ack"] = r["x"]
                cur["ack_source"] = r["ack_source"]
            continue
        if r["type"] == "STATUS" or pd.isna(r["state"]):
            continue
        state = r["state"]
        if state == "ALERT" and prev_state != "ALERT":
            # A person already present when the alert starts is whoever caused it
            # (e.g. dropped the box), not a responding driver: arrival is the first
            # NEW presence (a PRESENCE frame) after the alert starts and before the ACK.
            cur = {"start": r["x"], "arrive": None, "present_at_start": r.get("pir") == 1,
                   "ack": None, "ack_source": "", "end": None, "max_level": int(r["level"])}
            episodes.append(cur)
        elif cur is not None and state == "ALERT":
            cur["max_level"] = max(cur["max_level"], int(r["level"]))
            if cur["arrive"] is None and cur["ack"] is None and r["type"] == "PRESENCE":
                cur["arrive"] = r["x"]
        elif cur is not None and state != "ALERT":
            cur["end"] = r["x"]
            cur = None
        prev_state = state
    ev = d[d["type"] == "EVENT"]
    handling = int((ev["pir"] == 1).sum())
    return episodes, handling, len(ev) - handling


def _profile_starts(tel: pd.DataFrame):
    """(x, profile) at the start of each run of the same profile, for labelling limit lines."""
    out, prev = [], None
    for x, name in zip(tel["x"], tel["profile"].astype(str)):
        if name != prev and name in PROFILE_G_CRIT:
            out.append((x, name))
        prev = name
    return out


def style_axis(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color=TEXT, pad=6)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=TEXT_2, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def draw(fig, axes, d: pd.DataFrame, window_s: float, source: str):
    status_ax, risk_ax, shock_ax, tilt_ax, temp_ax, hum_ax = axes
    for ax in axes:
        ax.clear()
    status_ax.axis("off")

    if d.empty or not d["type"].isin(["TELEMETRY", "EVENT", "ALERT", "PRESENCE"]).any():
        status_ax.text(0.0, 0.5, f"Waiting for data in {source} (start pc/gateway.py) ...",
                       fontsize=13, color=TEXT_2, va="center")
        return

    x_end = d["x"].iloc[-1]
    x_start = max(0.0, x_end - window_s) if window_s > 0 else 0.0
    v = d[d["x"] >= x_start]
    tel = v[v["type"].isin(["TELEMETRY", "EVENT", "ALERT", "PRESENCE"])]
    last = d[d["type"].isin(["TELEMETRY", "EVENT", "ALERT", "PRESENCE"])].iloc[-1]
    level = int(last["level"])
    profile = str(last["profile"])
    episodes, n_handling, n_unattended = analyze(d)

    # ---------- status strip ----------
    status_ax.text(0.0, 0.62, f"{LEVEL_ICON[level]} {LEVEL_TEXT[level]}", fontsize=22,
                   fontweight="bold", color=STATUS[level], va="center")
    if last["state"] == "ALERT":
        ack_text = "waiting for driver ACK" if last["ack_pending"] == 1 else "ACKED (buzzer silenced)"
    else:
        ack_text = "no active alert"
    status_ax.text(0.0, 0.12, f"risk {int(last['risk'])}   |   {last['state']} - {ack_text}   |   "
                   f"profile {profile}", fontsize=11, color=TEXT, va="center")
    status_ax.text(0.42, 0.62, f"{last['temp_c']:.1f} °C   {last['hum_pct']:.0f} %RH   "
                   f"{last['press_hpa']:.1f} hPa", fontsize=11, color=TEXT, va="center")
    flags = [name for name, col in (("person nearby (PIR)", "pir"), ("box open (light)", "open"),
                                    ("accelerometer saturated", "sat")) if last.get(col) == 1]
    status_ax.text(0.42, 0.12, "flags: " + (", ".join(flags) if flags else "none"),
                   fontsize=11, color=TEXT_2, va="center")

    done = [e for e in episodes if e["ack"] is not None]
    if episodes:
        e = episodes[-1]
        if e["arrive"] is not None:
            arrive = f"{e['arrive'] - e['start']:.0f} s"
        elif e["present_at_start"]:
            arrive = "- (someone was already there)"
        else:
            arrive = "-"
        if e["ack"] is not None:
            ack = f"{e['ack'] - e['start']:.0f} s ({e['ack_source']})"
        elif e["end"] is not None:
            ack = "none - tilt corrected, cleared by itself"
        else:
            ack = "pending"
        line1 = f"last alert (L{e['max_level']}): person arrived {arrive}, ACK {ack}"
    else:
        line1 = "no alerts yet"
    n = len(episodes)
    line2 = f"{n} alert{'s' if n != 1 else ''}"
    if done:
        mean_ack = sum(e["ack"] - e["start"] for e in done) / len(done)
        line2 += f", mean time to ACK {mean_ack:.0f} s"
    line3 = f"shocks: {n_handling} during handling, {n_unattended} unattended"
    status_ax.text(0.70, 0.80, "Driver response", fontsize=11, fontweight="bold", color=TEXT, va="center")
    status_ax.text(0.70, 0.50, line1, fontsize=9, color=TEXT, va="center")
    status_ax.text(0.70, 0.24, line2, fontsize=9, color=TEXT_2, va="center")
    status_ax.text(0.70, -0.02, line3, fontsize=9, color=TEXT_2, va="center")

    # ---------- risk score with level bands ----------
    style_axis(risk_ax, "Risk score (edge-computed on the node) with alert bands")
    top = RISK_CAP
    for lo, hi, lvl in ((40, 70, 1), (70, 100, 2), (100, top, 3)):
        risk_ax.axhspan(lo, hi, color=STATUS[lvl], alpha=0.10, linewidth=0)
        risk_ax.text(x_start, lo + 2, f" {LEVEL_TEXT[lvl]} (≥{lo})", fontsize=8, color=TEXT_2, va="bottom")
    risk_ax.plot(tel["x"], tel["risk"].clip(upper=top), color=SERIES_1, linewidth=2, label="risk score")
    over = tel[tel["risk"] > top]
    if not over.empty:
        # Only a saturated shock is pinned at 999; an ordinary score above the axis is just clipped.
        note = f" score above {top}, clipped on chart (peak {int(over['risk'].max())})"
        if (over["risk"] >= 999).any():
            note += "; 999 = saturated shock"
        risk_ax.text(over["x"].iloc[0], top - 2, note, fontsize=8, color=TEXT_2, va="top")
    for _, r in v[v["type"] == "ACK_LOG"].iterrows():
        risk_ax.axvline(r["x"], color=TEXT, linewidth=1.5)
        risk_ax.text(r["x"], top * 0.84, f" ACK ({r['ack_source']})", fontsize=8, color=TEXT)
    for _, r in v[v["type"] == "PRESENCE"].iterrows():
        risk_ax.axvline(r["x"], color=TEXT_2, linewidth=1, linestyle=":")
        risk_ax.text(r["x"], top * 0.72, " person", fontsize=7, color=TEXT_2)
    risk_ax.set_ylim(0, top)
    risk_ax.set_xlim(x_start, max(x_start + 10, x_end))
    risk_ax.set_ylabel("score", fontsize=8, color=TEXT_2)

    # ---------- shock events ----------
    style_axis(shock_ax, "Shock events: peak acceleration")
    ev = v[v["type"] == "EVENT"]
    for pir_val, color, marker, label in ((1, SERIES_1, "o", "during handling (person present)"),
                                          (0, SERIES_2, "^", "unattended (vehicle)")):
        sub = ev[ev["pir"] == pir_val]
        if not sub.empty:
            shock_ax.vlines(sub["x"], 0, sub["peak_g"], color=color, linewidth=2)
            shock_ax.scatter(sub["x"], sub["peak_g"], color=color, marker=marker, s=48,
                             edgecolors=SURFACE, linewidths=2, zorder=3, label=label)
    # The profile can change during a run, so the limit is drawn as a step line over time.
    prof = tel["profile"].astype(str)
    g_line = prof.map(PROFILE_G_CRIT)
    if g_line.notna().any():
        shock_ax.plot(tel["x"], g_line, color=TEXT_2, linewidth=1, linestyle="--", drawstyle="steps-post")
        for x0, name in _profile_starts(tel):
            shock_ax.text(x0, PROFILE_G_CRIT[name], f" {name} limit {PROFILE_G_CRIT[name]:.0f} g",
                          fontsize=8, color=TEXT_2, va="bottom")
    shock_ax.set_ylim(0, 17)
    shock_ax.set_xlim(risk_ax.get_xlim())
    shock_ax.set_ylabel("g", fontsize=8, color=TEXT_2)
    if not ev.empty:
        shock_ax.legend(fontsize=8, frameon=False, loc="upper left")

    # ---------- tilt ----------
    style_axis(tilt_ax, "Tilt from resting orientation")
    tilt_ax.plot(tel["x"], tel["tilt_deg"], color=SERIES_1, linewidth=2)
    t_line = prof.map(PROFILE_TILT_LIMIT)
    if t_line.notna().any():
        tilt_ax.plot(tel["x"], t_line, color=TEXT_2, linewidth=1, linestyle="--", drawstyle="steps-post")
        for x0, name in _profile_starts(tel):
            tilt_ax.text(x0, PROFILE_TILT_LIMIT[name], f" {name} limit {PROFILE_TILT_LIMIT[name]}°",
                         fontsize=8, color=TEXT_2, va="bottom")
    tilt_ax.set_ylim(0, max(100, float(tel["tilt_deg"].max() or 0) + 10))
    tilt_ax.set_xlim(risk_ax.get_xlim())
    tilt_ax.set_ylabel("degrees", fontsize=8, color=TEXT_2)

    # ---------- environment (two panels, one axis each) ----------
    style_axis(temp_ax, "Temperature (DHT20)")
    env = tel[tel["temp_c"] != 0]
    temp_ax.plot(env["x"], env["temp_c"], color=SERIES_1, linewidth=2)
    temp_ax.set_xlim(risk_ax.get_xlim())
    temp_ax.set_ylabel("°C", fontsize=8, color=TEXT_2)
    temp_ax.set_xlabel("time since start (s)", fontsize=8, color=TEXT_2)

    style_axis(hum_ax, "Relative humidity (DHT20)")
    hum_ax.plot(env["x"], env["hum_pct"], color=SERIES_1, linewidth=2)
    hum_ax.set_xlim(risk_ax.get_xlim())
    hum_ax.set_ylabel("%RH", fontsize=8, color=TEXT_2)
    hum_ax.set_xlabel("time since start (s)", fontsize=8, color=TEXT_2)


def build_figure():
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(14, 8.6), facecolor=SURFACE)
    gs = fig.add_gridspec(4, 2, height_ratios=[0.55, 1.5, 1.1, 0.9], hspace=0.55, wspace=0.12,
                          left=0.05, right=0.98, top=0.95, bottom=0.07)
    status_ax = fig.add_subplot(gs[0, :])
    risk_ax = fig.add_subplot(gs[1, :])
    shock_ax = fig.add_subplot(gs[2, 0])
    tilt_ax = fig.add_subplot(gs[2, 1])
    temp_ax = fig.add_subplot(gs[3, 0])
    hum_ax = fig.add_subplot(gs[3, 1])
    fig.suptitle("ShipGuard - shipment condition dashboard (prototype: PC stands in for gateway + cloud)",
                 x=0.05, ha="left", fontsize=12, color=TEXT)
    return fig, (status_ax, risk_ax, shock_ax, tilt_ax, temp_ax, hum_ax)


def main():
    default_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "latest.csv")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=default_csv, help="gateway CSV log (default: pc/data/latest.csv)")
    ap.add_argument("--window", type=float, default=180, help="live view length in seconds (0 = all)")
    ap.add_argument("--static", metavar="PNG", help="render the whole log to an image and exit")
    args = ap.parse_args()

    if args.static:
        matplotlib.use("Agg")
        fig, axes = build_figure()
        draw(fig, axes, load(args.csv), 0, args.csv)
        fig.savefig(args.static, dpi=160, facecolor=SURFACE)
        print(f"saved {args.static}")
        return

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    fig, axes = build_figure()

    def update(_):
        draw(fig, axes, load(args.csv), args.window, args.csv)

    anim = FuncAnimation(fig, update, interval=1000, cache_frame_data=False)  # noqa: F841 (keep a reference)
    update(0)
    plt.show()


if __name__ == "__main__":
    main()
