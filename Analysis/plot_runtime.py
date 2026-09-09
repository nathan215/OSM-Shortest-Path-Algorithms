"""Plot mean runtime of own implementations vs NetworkX per algorithm, city and scenario.

Run from the repo root:  python Analysis/plot_runtime.py
Reads Analysis/algorithm_results_<city>_<scenario>.csv, writes Analysis/figures/runtime_own_vs_networkx.png
"""
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
CITIES = ["Taipei", "Taichung", "Kaohsiung", "Pingtung", "Hualien", "Taitung"]
SCENARIOS = [
    ("long_distance", "Long distance (≥ 5 km apart)"),
    ("short_distance", "Short distance (≤ 1 km, full city graph)"),
    ("short_distance_small_graph", "Short distance, small local graph"),
]
FAMILIES = [
    ("dijkstras", "nx_dijkstra", "Dijkstra"),
    ("bidirectional_dijkstra", "nx_bidirectional_dijkstra", "Bidirectional Dijkstra"),
    ("A_star", "nx_A_star", "A*"),
]
OWN, NX = "#2a78d6", "#eb6834"


def main() -> None:
    plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "figure.dpi": 160,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(3, 3, figsize=(11, 8.2), sharey=True)
    x, w = np.arange(len(CITIES)), 0.36
    for r, (key, label) in enumerate(SCENARIOS):
        for c, (own, nx, fam) in enumerate(FAMILIES):
            ax = axes[r, c]
            mo, mn = [], []
            for city in CITIES:
                df = pd.read_csv(HERE / f"algorithm_results_{city}_{key}.csv")
                mo.append(df[f"{own}_time"].mean() * 1000)
                mn.append(df[f"{nx}_time"].mean() * 1000)
            ax.bar(x - w / 2 - 0.01, mo, w, color=OWN, label="Own implementation", zorder=3)
            ax.bar(x + w / 2 + 0.01, mn, w, color=NX, label="NetworkX", zorder=3)
            ax.set_yscale("log"); ax.set_ylim(0.1, 200)
            ax.grid(axis="y", color="#e6e5e1", zorder=0); ax.set_axisbelow(True)
            ax.set_xticks(x); ax.set_xticklabels(CITIES, rotation=30, ha="right")
            if r == 0:
                ax.set_title(fam)
            if c == 0:
                ax.set_ylabel(f"{label}\nmean runtime (ms, log)", fontsize=8.5)
            if r == 1 and c == 0:
                for xi, (a, b) in enumerate(zip(mo, mn)):
                    if a / b > 10:
                        ax.annotate(f"{a / b:.0f}×", (xi, a), textcoords="offset points",
                                    xytext=(0, 3), ha="center", fontsize=7.5, color="#52514e")
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle("Shortest-path runtime on OpenStreetMap graphs: own implementations vs NetworkX "
                 "(mean over 100 random pairs per city)", y=1.035, fontsize=10.5)
    fig.tight_layout()
    out = HERE / "figures" / "runtime_own_vs_networkx.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
