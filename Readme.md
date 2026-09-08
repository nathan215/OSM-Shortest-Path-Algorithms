# OSM Shortest-Path Algorithms (HKUST UROP, 2023)

Benchmarking hand-written vs. NetworkX shortest-path algorithms on real OpenStreetMap road networks for six Taiwanese cities, plus a small Django + Leaflet app for interactive point-to-point routing.

## What it does

- **`Analysis/`** — an offline benchmark: downloads OSM drive-network graphs for six Taiwanese cities, generates random short- and long-distance point pairs on each, runs six shortest-path implementations on every pair, and records runtime and path cost to CSV.
- **`Uropapp/`** — a Django web app with a Leaflet/OpenStreetMap map UI: a user clicks a start and end point, the app queries a live OSM graph around those points, computes the route with the same six algorithms, and renders the chosen route(s) back onto the map as an image.

## Algorithms compared

Three algorithm families, each implemented twice — once by hand with `heapq`, once via NetworkX's built-in equivalent — so the benchmark is really "own implementation vs. library implementation":

| Family | Own implementation | NetworkX equivalent |
|---|---|---|
| Dijkstra | `dijkstras` | `nx_dijkstra` (`nx.single_source_dijkstra`) |
| Bidirectional Dijkstra | `bidirectional_dijkstra` | `nx_bidirectional_dijkstra` (`nx.bidirectional_dijkstra`) |
| A* | `A_star` (great-circle-distance heuristic) | `nx_A_star` (`nx.astar_path`) |

## Experiment design (`Analysis/main.py`)

- **Cities / graphs**: Taipei, Taichung, Kaohsiung, Pingtung, Hualien, Taitung. Each is a `drive`-network graph pulled from OSMnx around the city center with a 6 km radius (`Analysis/download_and_save_graphs.py`) and pickled to `Analysis/<city>_graph.pkl`. Reading those pickles directly gives:

  | City | Nodes | Edges |
  |---|---|---|
  | Taipei | 10,939 | 25,669 |
  | Taichung | 12,498 | 33,753 |
  | Kaohsiung | 10,163 | 28,963 |
  | Pingtung | 624 | 1,524 |
  | Hualien | 482 | 1,337 |
  | Taitung | 720 | 1,867 |

  i.e. three larger metro graphs (Taipei, Taichung, Kaohsiung) and three much smaller ones (Pingtung, Hualien, Taitung) — this is the "complex" vs. "single" city split `main.py` sets up. (`Analysis/areaselect.py` is a separate, throwaway sizing script that samples a wider 60 km-radius graph per city just to gauge relative road-network complexity; it isn't the source of the graphs actually used in the benchmark.)
- **Scenarios**, 100 random point pairs generated per city per scenario:
  - `long_distance` — two random nodes at least 5 km apart (`select_point_long`).
  - `short_distance` — two random nodes within 4 km of the city center and no more than 1 km apart (`select_point_short`).
  - `short_distance_small_graph` — the same kind of short-distance pair, but each pair is run on a small graph freshly queried from OSM around just those two points, rather than on the full city graph.
- **Metrics recorded**, per algorithm per pair, written to `Analysis/algorithm_results_<city>_<scenario>.csv`: wall-clock `time` (seconds, via `time.perf_counter`) and path `cost` (meters, summed edge `length`). Nodes explored is not recorded.

## Results

Example — mean over the 99 completed pairs (of the 100 requested) in `Analysis/algorithm_results_Taipei_short_distance.csv`:

| Algorithm | Mean time (ms) | Mean path cost (m) |
|---|---|---|
| `bidirectional_dijkstra` (own) | 26.51 | 1092.23 |
| `nx_bidirectional_dijkstra` | 0.57 | 1069.84 |
| `dijkstras` (own) | 58.56 | 1069.84 |
| `nx_dijkstra` | 0.96 | 1069.84 |
| `A_star` (own) | 6.70 | 1069.84 |
| `nx_A_star` | 1.15 | 1069.84 |

NetworkX's built-ins are roughly 6-60x faster here than the hand-written versions (A* is the closest pairing, bidirectional and plain Dijkstra the widest gaps), and all six methods agree on the optimal path cost except the hand-written `bidirectional_dijkstra`, which averages about 2% higher — consistent with its early-stop condition occasionally cutting the search short. There are 18 result CSVs in total (6 cities x 3 scenarios) to check whether this pattern holds elsewhere; numbers above are computed directly from the one file named, not aggregated across files.

## How to run

### Analysis benchmark

```bash
cd Analysis
pip install osmnx networkx geopy
python download_and_save_graphs.py   # optional: only if you want to refresh the .pkl graphs from OpenStreetMap
python main.py                       # runs all scenarios and writes algorithm_results_*.csv
```

`main.py` loads the six `*_graph.pkl` files already committed in this folder, so `download_and_save_graphs.py` only needs to be re-run if you want fresh data.

### Django app

```bash
pip install django osmnx networkx geopy matplotlib numpy
python manage.py runserver
```

`manage.py` sets `DJANGO_SETTINGS_MODULE=Urop.settings`, but the `Urop/` project package (`settings.py`, `wsgi.py`, etc.) is listed in `.gitignore` and is not in this repo, so `runserver` will not work until you scaffold it yourself, e.g. `django-admin startproject Urop .`, register `Uropapp` as an installed app, and wire its `urls.py` into the project's URL config.

## Repo layout

```
Analysis/
  areaselect.py                              # 60km-radius node/edge-count scouting per city
  download_and_save_graphs.py                # downloads + pickles the 6km-radius drive graph per city
  main.py                                    # 6 algorithms x 3 scenarios x 100 pairs x 6 cities benchmark
  <city>_graph.pkl                           # pickled networkx MultiDiGraph per city
  algorithm_results_<city>_<scenario>.csv    # benchmark output, one file per city/scenario
  dijkstras_profiling.txt
Uropapp/
  views.py             # CalculatePathView (runs the 6 algorithms on a live OSM query) and ShowPathsView (renders the chosen route(s) as a PNG)
  path_calculators.py  # the 6 shortest-path implementations used by the web app
  globals.py           # shared graph G plus init_graph() to load it from a center point + radius
  urls.py, models.py, apps.py, admin.py, tests.py, __init__.py
  home.html            # Leaflet/OpenStreetMap page for picking start/end points
manage.py
```

The `.pkl` graphs and `algorithm_results_*.csv` files are committed as-is so the benchmark results can be inspected, or `Analysis/main.py` re-run, without needing to re-download road-network data from OpenStreetMap.
