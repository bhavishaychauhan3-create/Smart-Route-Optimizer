# Smart Navigation & Algorithm Visualization Platform

A full-featured Python & Flask platform combining real-world multi-modal road routing (Driving, Cycling, Walking) with an interactive **Algorithm Visualization & Comparison Lab** (Dijkstra vs A*), multi-stop TSP delivery tour optimizer, live address autocomplete, carbon footprint analytics, and GPS file export capabilities (GPX / GeoJSON).

---

## 🌟 Key Features

### 1. Algorithm Visualizer & Comparison Lab (`/visualizer`)
- **Step-by-Step Exploration:** Real-time playback of node expansion in a 2D mesh grid with obstacles.
- **Node State Highlights:**
  - 🟡 **Frontier / Open Set** (nodes discovered in priority queue)
  - 🔵 **Visited / Closed Set** (settled nodes)
  - 🟣 **Current Node** (active node being evaluated)
  - 🟢 **Final Shortest Path** (backtracked optimal trajectory)
  - ⬛ **Obstacles / Barriers**
- **Interactive Controls:** Start Visualization, Pause, Resume, Reset, and Speed Slider (1x to 10x).
- **Side-by-Side Comparison Benchmark:** Compares Dijkstra vs A* on identical obstacle networks, tracking:
  - Execution time in microseconds (`µs`)
  - Nodes explored & exploration reduction percentage (`%`)
  - Memory allocation estimates in bytes
  - Shortest path distance verification

### 2. Multi-Modal Road Navigation (`/`)
- **Transport Modes:**
  - 🚗 **Driving:** Standard road network routing with car speed profiles (~45 km/h).
  - 🚲 **Cycling:** Road network routing with cycling speed and calorie estimation (~16 km/h).
  - 🚶 **Walking:** Pedestrian navigation with walking speed profiles (~4.8 km/h).
- **Intelligent Address Autocomplete:** Real-time debounced location suggestions as you type, powered by the Photon API and Nominatim.
- **Defensive Geolocation:** Automatic browser GPS detection with manual fallback handling.

### 3. Multi-Stop Delivery Optimizer (TSP)
- Solves the Traveling Salesperson Problem on 3+ stops using:
  - Nearest Neighbor heuristic initialization.
  - 2-Opt local search heuristic for route untangling.
- Displays savings in distance (`km`) and percentage reduction (`%`).

### 4. Route Analytics Dashboard
- **Trip Metrics:** Source, Destination, Waypoint count, Distance, and ETA.
- **Environmental & Energy Footprint:**
  - Estimated fuel consumption (Liters)
  - Carbon footprint (kg of CO₂ emissions)
  - Active calories burned (cycling & walking)
  - Trees needed to offset the carbon footprint

### 5. Data Export & Printing
- 📥 **Download GPX:** Standard XML GPS Exchange Format 1.1 file with trackpoints and waypoints.
- 📥 **Download GeoJSON:** RFC 7946 GeoJSON FeatureCollection with LineString and Point features.
- 🖨️ **Print Route Summary:** Formatted printable turn-by-turn driving manifest.

### 6. Modern UI/UX & Dark Mode
- Built with CSS variables supporting instant, persistent Dark/Light mode toggles.
- Responsive design adapting between desktop split-view and mobile layout.

---

## 🏗️ Architecture

```
project1/
├── app.py                      # Flask Application Controller, routing & REST API
├── graph_engine.py             # Graph data structures, Dijkstra, A*, TSP & Exporters
├── requirements.txt            # Project dependencies
├── .gitignore                  # Git exclusions (caches, environments)
├── README.md                   # Complete documentation
│
├── templates/
│   ├── index.html              # Search interface with autocomplete & mode toggles
│   ├── map.html                # Leaflet split-view dashboard & analytics
│   └── visualizer.html         # Algorithm Visualizer & Comparison Lab
│
└── tests/
    └── test_routing.py         # Unit & integration test suite (15 tests)
```

---

## 🚀 Getting Started

### 1. Installation
```bash
pip install -r requirements.txt
```

### 2. Running the Development Server
```bash
python app.py
```
Visit in your web browser:
- Main Navigation: `http://127.0.0.1:5000/`
- Algorithm Visualizer: `http://127.0.0.1:5000/visualizer`

---

## 🧪 Running Automated Tests

Run the built-in 15-test verification suite (zero external test dependencies required):
```bash
python -m unittest tests/test_routing.py -v
```

---

## 📡 REST API Reference

| Endpoint | Method | Description |
| :--- | :---: | :--- |
| `POST /api/route` | `POST` | Calculate route polyline, distance, and duration between coordinates. |
| `POST /api/optimize` | `POST` | Solve TSP multi-stop sequence for an array of coordinate points. |
| `GET /api/autocomplete` | `GET` | Instant search suggestions (`?q=term`). |
| `POST /api/visualize` | `POST` | Returns animated step-by-step frame trace for Dijkstra / A*. |
| `POST /api/compare` | `POST` | Returns side-by-side benchmark metrics (time, nodes, memory, efficiency). |
| `POST /api/export/gpx` | `POST` | Download route as `.gpx` file. |
| `POST /api/export/geojson`| `POST` | Download route as `.geojson` file. |

---

## 📄 License
MIT License.
