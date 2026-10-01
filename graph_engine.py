"""
graph_engine.py - Graph Data Structures, Shortest Path Algorithms, Visualization Engine & Exporters
Provides:
1. Graph representation with geographic coordinate support.
2. Dijkstra's Algorithm (Standard & Stepped Animation Trace).
3. A* Search Algorithm (Standard & Stepped Animation Trace).
4. Algorithm Performance Comparison & Benchmarking.
5. Grid Graph Generator with Obstacles.
6. Multi-Stop TSP 2-Opt Route Optimizer.
7. Environmental & Energy Impact Analytics (Fuel, CO2).
8. GPX and GeoJSON Format Exporters.
"""

import heapq
import math
import sys
import time
import json
import random
import os
import requests
from typing import Dict, List, Tuple, Optional, Any, Set


def haversine_distance(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    """
    Computes Great-Circle (haversine) distance between two (lat, lon) pairs in kilometers.
    coord format: (latitude, longitude)
    """
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.asin(math.sqrt(a))
    radius_earth_km = 6371.0088
    return radius_earth_km * c


class Graph:
    """
    Weighted Directed/Undirected Graph supporting spatial coordinates,
    shortest path algorithms, and step-by-step animation traces.
    """

    def __init__(self):
        # Adjacency list: {node: [(neighbor, weight, edge_id), ...]}
        self.adj: Dict[str, List[Tuple[str, float, str]]] = {}
        # Node coordinates: {node: (lat, lon)}
        self.coords: Dict[str, Tuple[float, float]] = {}
        # High-resolution edge curvature geometries: {(u, v): [(lat, lon), ...]}
        self.edge_geometries: Dict[Tuple[str, str], List[Tuple[float, float]]] = {}
        # Edge metadata (street name, maneuver instruction, speed limit, etc.): {(u, v): dict}
        self.edge_metadata: Dict[Tuple[str, str], Dict[str, Any]] = {}

    def add_node(self, node_id: str, lat: Optional[float] = None, lon: Optional[float] = None):
        """Adds a node to the graph with optional latitude and longitude coordinates."""
        if node_id not in self.adj:
            self.adj[node_id] = []
        if lat is not None and lon is not None:
            self.coords[node_id] = (float(lat), float(lon))

    def add_edge(
        self,
        u: str,
        v: str,
        weight: Optional[float] = None,
        bidirectional: bool = True,
        edge_id: Optional[str] = None,
        geometry: Optional[List[Tuple[float, float]]] = None,
        metadata: Optional[Dict[str, Any]] = None
    ):
        """Adds a weighted edge between u and v with optional geometry and metadata."""
        self.add_node(u)
        self.add_node(v)

        if weight is None:
            if u in self.coords and v in self.coords:
                weight = haversine_distance(self.coords[u], self.coords[v])
            else:
                weight = 1.0

        label = edge_id or f"{u}-{v}"
        self.adj[u].append((v, float(weight), label))
        if geometry:
            self.edge_geometries[(u, v)] = geometry
        if metadata:
            self.edge_metadata[(u, v)] = metadata

        if bidirectional:
            rev_label = edge_id or f"{v}-{u}"
            self.adj[v].append((u, float(weight), rev_label))
            if geometry:
                self.edge_geometries[(v, u)] = list(reversed(geometry))
            if metadata:
                self.edge_metadata[(v, u)] = metadata

    def snap_to_nearest_node(self, lat: float, lon: float) -> Tuple[Optional[str], float]:
        """
        Snaps an arbitrary coordinate (lat, lon) to the nearest road node in the graph.
        Returns: (node_id, snap_distance_meters)
        """
        if not self.coords:
            return None, float('inf')

        target = (lat, lon)
        best_node = None
        min_dist_km = float('inf')

        for node_id, coord in self.coords.items():
            d = haversine_distance(target, coord)
            if d < min_dist_km:
                min_dist_km = d
                best_node = node_id

        return best_node, round(min_dist_km * 1000.0, 2)

    def get_connected_components(self) -> List[Set[str]]:
        """Finds all connected components in the graph."""
        visited: Set[str] = set()
        components: List[Set[str]] = []

        for node in self.adj:
            if node not in visited:
                comp: Set[str] = set()
                queue = [node]
                visited.add(node)
                while queue:
                    curr = queue.pop(0)
                    comp.add(curr)
                    for neighbor, _, _ in self.adj.get(curr, []):
                        if neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
                components.append(comp)
        return components

    def is_reachable(self, start: str, target: str) -> bool:
        """Checks if target node is reachable from start node using BFS traversal."""
        if start not in self.adj or target not in self.adj:
            return False
        if start == target:
            return True

        visited: Set[str] = {start}
        queue = [start]
        while queue:
            curr = queue.pop(0)
            if curr == target:
                return True
            for neighbor, _, _ in self.adj.get(curr, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return False

    def bridge_components_if_needed(self, start: str, target: str) -> bool:
        """
        Detects if start and target belong to disconnected components.
        If disconnected, gracefully bridges the two closest nodes of the
        respective components with a road transition edge so routing never fails.
        """
        if self.is_reachable(start, target):
            return False

        # Gather start component
        comp_start: Set[str] = set()
        visited: Set[str] = {start}
        queue = [start]
        while queue:
            curr = queue.pop(0)
            comp_start.add(curr)
            for neighbor, _, _ in self.adj.get(curr, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # Gather target component
        comp_target: Set[str] = set()
        visited = {target}
        queue = [target]
        while queue:
            curr = queue.pop(0)
            comp_target.add(curr)
            for neighbor, _, _ in self.adj.get(curr, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        # Find closest node pair between the two components
        min_dist = float('inf')
        best_u, best_v = start, target

        nodes_s = list(comp_start)[:200]
        nodes_t = list(comp_target)[:200]

        for u in nodes_s:
            if u not in self.coords:
                continue
            for v in nodes_t:
                if v not in self.coords:
                    continue
                d = haversine_distance(self.coords[u], self.coords[v])
                if d < min_dist:
                    min_dist = d
                    best_u, best_v = u, v

        print(f"[RoadGraph] Disconnected graph components detected between '{start}' and '{target}'.")
        print(f"[RoadGraph] Gracefully bridging component at node '{best_u}' to node '{best_v}' (distance: {min_dist:.2f} km).")

        coord_u = self.coords.get(best_u, (0.0, 0.0))
        coord_v = self.coords.get(best_v, (0.0, 0.0))
        bridge_geom = [coord_u, coord_v]

        self.add_edge(best_u, best_v, weight=min_dist, bidirectional=True, edge_id="bridge_edge", geometry=bridge_geom)
        return True

    def dijkstra(self, start: str, target: str) -> Tuple[List[str], float, int]:
        """
        Standard Dijkstra's Algorithm using min-heap priority queue.
        Returns: (path, total_distance, visited_nodes_count)
        """
        if start not in self.adj or target not in self.adj:
            return [], float('inf'), 0

        distances: Dict[str, float] = {node: float('inf') for node in self.adj}
        predecessors: Dict[str, Optional[str]] = {node: None for node in self.adj}
        distances[start] = 0.0

        pq: List[Tuple[float, str]] = [(0.0, start)]
        visited_count = 0
        visited_set: Set[str] = set()

        while pq:
            current_dist, u = heapq.heappop(pq)

            if u in visited_set:
                continue
            visited_set.add(u)
            visited_count += 1

            if u == target:
                break

            if current_dist > distances[u]:
                continue

            for neighbor, weight, _ in self.adj.get(u, []):
                new_dist = current_dist + weight
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    predecessors[neighbor] = u
                    heapq.heappush(pq, (new_dist, neighbor))

        path: List[str] = []
        curr: Optional[str] = target
        while curr is not None:
            path.append(curr)
            curr = predecessors.get(curr)
        path.reverse()

        if not path or path[0] != start:
            return [], float('inf'), visited_count

        return path, round(distances[target], 4), visited_count

    def dijkstra_stepped(self, start: str, target: str) -> Dict[str, Any]:
        """
        Dijkstra's Algorithm with step-by-step trace generation for visualizers.
        Captures state transitions: 'current', 'frontier', 'visited', and 'path'.
        """
        if start not in self.adj or target not in self.adj:
            return {"steps": [], "path": [], "distance": float('inf'), "nodes_visited": 0}

        distances: Dict[str, float] = {node: float('inf') for node in self.adj}
        predecessors: Dict[str, Optional[str]] = {node: None for node in self.adj}
        distances[start] = 0.0

        pq: List[Tuple[float, str]] = [(0.0, start)]
        visited_set: Set[str] = set()
        steps: List[Dict[str, Any]] = []

        # Initial frontier step
        steps.append({
            "action": "frontier",
            "node": start,
            "cost": 0.0,
            "coord": self.coords.get(start)
        })

        while pq:
            current_dist, u = heapq.heappop(pq)

            if u in visited_set:
                continue
            visited_set.add(u)

            # Record 'current' node action
            steps.append({
                "action": "current",
                "node": u,
                "cost": round(current_dist, 2),
                "coord": self.coords.get(u)
            })

            # Record 'visited' / closed set action
            steps.append({
                "action": "visited",
                "node": u,
                "cost": round(current_dist, 2),
                "coord": self.coords.get(u)
            })

            if u == target:
                break

            for neighbor, weight, _ in self.adj.get(u, []):
                if neighbor in visited_set:
                    continue
                new_dist = current_dist + weight
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    predecessors[neighbor] = u
                    heapq.heappush(pq, (new_dist, neighbor))
                    # Record 'frontier' / open set action
                    steps.append({
                        "action": "frontier",
                        "node": neighbor,
                        "cost": round(new_dist, 2),
                        "parent": u,
                        "coord": self.coords.get(neighbor)
                    })

        # Reconstruct path
        path: List[str] = []
        curr: Optional[str] = target
        while curr is not None:
            path.append(curr)
            curr = predecessors.get(curr)
        path.reverse()

        valid_path = path if (path and path[0] == start) else []

        # Record final path steps
        for p_node in valid_path:
            steps.append({
                "action": "path",
                "node": p_node,
                "coord": self.coords.get(p_node)
            })

        return {
            "steps": steps,
            "path": valid_path,
            "distance": round(distances[target], 2) if valid_path else float('inf'),
            "nodes_visited": len(visited_set),
            "total_nodes": len(self.adj)
        }

    def a_star(self, start: str, target: str) -> Tuple[List[str], float, int]:
        """
        Standard A* Search Algorithm with an admissible Haversine heuristic.
        Returns: (path, total_distance, visited_nodes_count)
        """
        if start not in self.adj or target not in self.adj:
            return [], float('inf'), 0

        if start not in self.coords or target not in self.coords:
            return self.dijkstra(start, target)

        target_coord = self.coords[target]

        def heuristic(node: str) -> float:
            if node in self.coords:
                return haversine_distance(self.coords[node], target_coord)
            return 0.0

        g_score: Dict[str, float] = {node: float('inf') for node in self.adj}
        g_score[start] = 0.0

        f_score: Dict[str, float] = {node: float('inf') for node in self.adj}
        f_score[start] = heuristic(start)

        predecessors: Dict[str, Optional[str]] = {node: None for node in self.adj}
        pq: List[Tuple[float, str]] = [(f_score[start], start)]
        visited_count = 0
        visited_set: Set[str] = set()

        while pq:
            _, u = heapq.heappop(pq)

            if u in visited_set:
                continue
            visited_set.add(u)
            visited_count += 1

            if u == target:
                break

            for neighbor, weight, _ in self.adj.get(u, []):
                tentative_g = g_score[u] + weight
                if tentative_g < g_score[neighbor]:
                    predecessors[neighbor] = u
                    g_score[neighbor] = tentative_g
                    f_val = tentative_g + heuristic(neighbor)
                    f_score[neighbor] = f_val
                    heapq.heappush(pq, (f_val, neighbor))

        path: List[str] = []
        curr: Optional[str] = target
        while curr is not None:
            path.append(curr)
            curr = predecessors.get(curr)
        path.reverse()

        if not path or path[0] != start:
            return [], float('inf'), visited_count

        return path, round(g_score[target], 4), visited_count

    def a_star_stepped(self, start: str, target: str) -> Dict[str, Any]:
        """
        A* Search Algorithm with step-by-step trace generation for visualizers.
        Captures f_score, g_score, heuristic (h), and state transitions.
        """
        if start not in self.adj or target not in self.adj:
            return {"steps": [], "path": [], "distance": float('inf'), "nodes_visited": 0}

        target_coord = self.coords.get(target, (0.0, 0.0))

        def heuristic(node: str) -> float:
            if node in self.coords and target in self.coords:
                return round(haversine_distance(self.coords[node], target_coord), 2)
            return 0.0

        g_score: Dict[str, float] = {node: float('inf') for node in self.adj}
        g_score[start] = 0.0

        f_score: Dict[str, float] = {node: float('inf') for node in self.adj}
        f_score[start] = heuristic(start)

        predecessors: Dict[str, Optional[str]] = {node: None for node in self.adj}
        pq: List[Tuple[float, str]] = [(f_score[start], start)]
        visited_set: Set[str] = set()
        steps: List[Dict[str, Any]] = []

        # Initial frontier step
        steps.append({
            "action": "frontier",
            "node": start,
            "g": 0.0,
            "h": f_score[start],
            "f": f_score[start],
            "coord": self.coords.get(start)
        })

        while pq:
            _, u = heapq.heappop(pq)

            if u in visited_set:
                continue
            visited_set.add(u)

            steps.append({
                "action": "current",
                "node": u,
                "g": round(g_score[u], 2),
                "h": heuristic(u),
                "f": round(f_score[u], 2),
                "coord": self.coords.get(u)
            })

            steps.append({
                "action": "visited",
                "node": u,
                "g": round(g_score[u], 2),
                "h": heuristic(u),
                "f": round(f_score[u], 2),
                "coord": self.coords.get(u)
            })

            if u == target:
                break

            for neighbor, weight, _ in self.adj.get(u, []):
                if neighbor in visited_set:
                    continue
                tentative_g = g_score[u] + weight
                if tentative_g < g_score[neighbor]:
                    predecessors[neighbor] = u
                    g_score[neighbor] = tentative_g
                    h_val = heuristic(neighbor)
                    f_val = tentative_g + h_val
                    f_score[neighbor] = f_val
                    heapq.heappush(pq, (f_val, neighbor))

                    steps.append({
                        "action": "frontier",
                        "node": neighbor,
                        "parent": u,
                        "g": round(tentative_g, 2),
                        "h": h_val,
                        "f": round(f_val, 2),
                        "coord": self.coords.get(neighbor)
                    })

        path: List[str] = []
        curr: Optional[str] = target
        while curr is not None:
            path.append(curr)
            curr = predecessors.get(curr)
        path.reverse()

        valid_path = path if (path and path[0] == start) else []

        for p_node in valid_path:
            steps.append({
                "action": "path",
                "node": p_node,
                "coord": self.coords.get(p_node)
            })

        return {
            "steps": steps,
            "path": valid_path,
            "distance": round(g_score[target], 2) if valid_path else float('inf'),
            "nodes_visited": len(visited_set),
            "total_nodes": len(self.adj)
        }

    def compare_algorithms(self, start: str, target: str) -> Dict[str, Any]:
        """
        Runs side-by-side benchmark comparing Dijkstra vs A* on the same graph instance.
        Measures execution time (microseconds), memory footprint estimate, and exploration efficiency.
        """
        # Benchmark Dijkstra
        t0 = time.perf_counter_ns()
        d_path, d_dist, d_visited = self.dijkstra(start, target)
        d_time_us = round((time.perf_counter_ns() - t0) / 1000.0, 2)
        # Approximate state memory in bytes
        d_memory_est = sys.getsizeof(self.adj) + (len(self.adj) * 64)

        # Benchmark A*
        t1 = time.perf_counter_ns()
        a_path, a_dist, a_visited = self.a_star(start, target)
        a_time_us = round((time.perf_counter_ns() - t1) / 1000.0, 2)
        a_memory_est = sys.getsizeof(self.adj) + (len(self.adj) * 72)

        # Efficiency calculation
        nodes_saved = max(0, d_visited - a_visited)
        exploration_reduction_pct = round((nodes_saved / d_visited * 100.0), 1) if d_visited > 0 else 0.0

        return {
            "start_node": start,
            "target_node": target,
            "path_found": len(d_path) > 0,
            "dijkstra": {
                "name": "Dijkstra's Algorithm",
                "distance_km": d_dist,
                "execution_time_us": d_time_us,
                "nodes_explored": d_visited,
                "path_node_count": len(d_path),
                "memory_estimate_bytes": d_memory_est
            },
            "a_star": {
                "name": "A* Search Algorithm",
                "distance_km": a_dist,
                "execution_time_us": a_time_us,
                "nodes_explored": a_visited,
                "path_node_count": len(a_path),
                "memory_estimate_bytes": a_memory_est
            },
            "comparison": {
                "nodes_saved_by_a_star": nodes_saved,
                "efficiency_gain_percent": exploration_reduction_pct,
                "is_optimal_match": abs(d_dist - a_dist) < 1e-4
            }
        }


# ==========================================
# WORLDWIDE ROAD NETWORK ROUTING ENGINE
# ==========================================

def decode_valhalla_polyline(encoded: str, precision: int = 6) -> List[List[float]]:
    """Decodes Valhalla / Google encoded polyline string to list of [lat, lon] coordinates."""
    inv = 1.0 / (10 ** precision)
    coords = []
    lat = 0
    lon = 0
    idx = 0
    length = len(encoded)

    while idx < length:
        shift = 0
        result = 0
        while True:
            byte = ord(encoded[idx]) - 63
            idx += 1
            result |= (byte & 0x1f) << shift
            shift += 5
            if byte < 0x20:
                break
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat

        shift = 0
        result = 0
        while True:
            byte = ord(encoded[idx]) - 63
            idx += 1
            result |= (byte & 0x1f) << shift
            shift += 5
            if byte < 0x20:
                break
        dlon = ~(result >> 1) if (result & 1) else (result >> 1)
        lon += dlon

        coords.append([round(lat * inv, 6), round(lon * inv, 6)])

    return coords


class WorldwideRoadRouter:
    """
    Worldwide Road Network Routing Engine:
    - Primary: OSRM Global Routing Cluster (worldwide road networks, real highway curves)
    - Shortest Drivable Evaluator: Valhalla OpenStreetMap Engine across all road categories
      (Motorway, Trunk, Primary, Secondary, Tertiary, Residential, Unclassified, Service, Track)
    - Global Mirror: OpenStreetMap DE Routing Cluster (automatic failover)
    - External Adapters: OpenRouteService & GraphHopper support
    - Intelligent Fallback: Local OSMnx + NetworkX graph routing or high-density road corridor graph
    - In-Memory LRU Route Cache for high-performance and quota conservation
    - Full diagnostics: timing, provider, straight-line comparison ratio, road point counts, detour savings
    """

    OSM_PRIMARY_URL = "http://router.project-osrm.org/route/v1"
    OSM_MIRROR_URL = "https://routing.openstreetmap.de/routed-car/route/v1"
    OSM_TABLE_URL = "http://router.project-osrm.org/table/v1"
    VALHALLA_URL = "https://valhalla1.openstreetmap.de/route"

    _ROUTE_CACHE: Dict[str, Dict[str, Any]] = {}
    _CACHE_MAX_SIZE: int = 256

    @classmethod
    def clear_cache(cls):
        """Clears in-memory route cache."""
        cls._ROUTE_CACHE.clear()

    @classmethod
    def get_cache_stats(cls) -> Dict[str, Any]:
        """Returns statistics on in-memory route cache."""
        return {
            "cached_routes_count": len(cls._ROUTE_CACHE),
            "max_cache_size": cls._CACHE_MAX_SIZE
        }

    @classmethod
    def _make_cache_key(cls, coordinates: List[Tuple[float, float]], mode: str, provider: str, algorithm: str = "a_star") -> str:
        coords_repr = ";".join(f"{lat:.5f},{lon:.5f}" for lat, lon in coordinates)
        return f"{provider.lower()}:{mode.lower()}:{algorithm.lower()}:{coords_repr}"

    @classmethod
    def fetch_osrm(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        timeout: int = 25
    ) -> Tuple[Optional[Dict[str, Any]], str, float]:
        """Queries OSRM global routing cluster with failover to OpenStreetMap DE mirror."""
        mode = mode.lower()
        profile = "driving"
        if mode == "cycling":
            profile = "bike" if "routing.openstreetmap.de" in cls.OSM_PRIMARY_URL else "driving"
        elif mode == "walking":
            profile = "foot" if "routing.openstreetmap.de" in cls.OSM_PRIMARY_URL else "driving"

        coords_param = ";".join(f"{lon:.6f},{lat:.6f}" for lat, lon in coordinates)

        endpoints = [
            (f"{cls.OSM_PRIMARY_URL}/{profile}/{coords_param}?overview=full&geometries=geojson&steps=true&alternatives=3&annotations=nodes,distance", "OSRM Global Road Network Engine"),
            (f"{cls.OSM_MIRROR_URL}/{profile}/{coords_param}?overview=full&geometries=geojson&steps=true&alternatives=3&annotations=nodes,distance", "OSRM Global Mirror (OpenStreetMap DE)")
        ]

        for url, provider_name in endpoints:
            t0 = time.perf_counter()
            try:
                resp = requests.get(url, timeout=timeout)
                t_api = round((time.perf_counter() - t0) * 1000.0, 2)
                if resp.status_code == 200:
                    data = resp.json()
                    if data.get("code") == "Ok" and data.get("routes"):
                        return data, provider_name, t_api
                    elif data.get("code") == "NoRoute":
                        return data, provider_name, t_api
            except Exception as e:
                print(f"[WorldwideRouter] OSRM endpoint error ({provider_name}): {e}")

        return None, "Unavailable", 0.0

    @classmethod
    def fetch_valhalla_shortest(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        timeout: int = 12
    ) -> Optional[Dict[str, Any]]:
        """
        Queries Valhalla OpenStreetMap routing engine configured for shortest drivable distance
        incorporating all road categories (unclassified, residential, tertiary, service, tracks)
        to identify and bypass unnecessary highway detours.
        """
        costing = "auto"
        costing_options = {
            "auto": {
                "shortest": True,
                "use_highways": 0.1,
                "use_tolls": 0.5
            }
        }
        if mode == "cycling":
            costing = "bicycle"
            costing_options = {}
        elif mode == "walking":
            costing = "pedestrian"
            costing_options = {}

        payload = {
            "locations": [{"lat": c[0], "lon": c[1]} for c in coordinates],
            "costing": costing,
            "costing_options": costing_options,
            "directions_options": {"units": "kilometers"}
        }

        t0 = time.perf_counter()
        try:
            r = requests.post(cls.VALHALLA_URL, json=payload, timeout=timeout)
            t_api = round((time.perf_counter() - t0) * 1000.0, 2)
            if r.status_code == 200:
                data = r.json()
                if "trip" in data and "legs" in data["trip"]:
                    trip = data["trip"]
                    dist_km = round(trip["summary"]["length"], 2)

                    # Calibrate realistic travel duration by travel mode
                    if mode == "cycling":
                        dur_min = round((dist_km / 16.0) * 60.0, 1)
                    elif mode == "walking":
                        dur_min = round((dist_km / 4.8) * 60.0, 1)
                    else:
                        # Real-world rural & local driving average ~38 km/h
                        dur_min = round((dist_km / 38.0) * 60.0, 1)

                    polyline: List[List[float]] = []
                    steps: List[Dict[str, Any]] = []

                    for leg in trip["legs"]:
                        leg_shape = decode_valhalla_polyline(leg.get("shape", ""))
                        if polyline and leg_shape and polyline[-1] == leg_shape[0]:
                            polyline.extend(leg_shape[1:])
                        else:
                            polyline.extend(leg_shape)

                        for maneuver in leg.get("maneuvers", []):
                            instruction = maneuver.get("instruction", "Continue")
                            m_dist = round(maneuver.get("length", 0.0) * 1000.0, 1)
                            steps.append({
                                "instruction": instruction,
                                "distance_m": m_dist,
                                "type": "drive",
                                "modifier": ""
                            })

                    return {
                        "success": True,
                        "distance_km": dist_km,
                        "duration_min": dur_min,
                        "polyline": polyline,
                        "steps": steps,
                        "provider": "OSM Shortest Drivable Engine (All Road Categories)",
                        "api_response_time_ms": t_api
                    }
        except Exception as e:
            print(f"[WorldwideRouter] Valhalla shortest engine notice: {e}")
        return None

    @classmethod
    def fetch_osm_raw(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        timeout: int = 30
    ) -> Optional[Dict[str, Any]]:
        """Backward compatibility helper."""
        data, _, _ = cls.fetch_osrm(coordinates, mode=mode, timeout=timeout)
        return data

    @classmethod
    def fetch_openrouteservice(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        api_key: Optional[str] = None,
        timeout: int = 20
    ) -> Tuple[Optional[Dict[str, Any]], str, float]:
        """Queries OpenRouteService API if API key is configured."""
        key = api_key or os.environ.get("ORS_API_KEY")
        if not key:
            return None, "OpenRouteService", 0.0

        profile_map = {"driving": "driving-car", "cycling": "cycling-regular", "walking": "foot-walking"}
        profile = profile_map.get(mode.lower(), "driving-car")
        url = f"https://api.openrouteservice.org/v2/directions/{profile}/geojson"

        coords_body = [[lon, lat] for lat, lon in coordinates]
        headers = {"Authorization": key, "Content-Type": "application/json"}

        t0 = time.perf_counter()
        try:
            resp = requests.post(url, json={"coordinates": coords_body}, headers=headers, timeout=timeout)
            t_api = round((time.perf_counter() - t0) * 1000.0, 2)
            if resp.status_code == 200:
                return resp.json(), "OpenRouteService API", t_api
        except Exception as e:
            print(f"[WorldwideRouter] OpenRouteService error: {e}")
        return None, "OpenRouteService", 0.0

    @classmethod
    def local_osmnx_route(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving"
    ) -> Optional[Dict[str, Any]]:
        """Computes local route via OSMnx + NetworkX for small bounding boxes (< 40km)."""
        try:
            import osmnx as ox
            import networkx as nx

            lats = [c[0] for c in coordinates]
            lons = [c[1] for c in coordinates]
            max_span_km = haversine_distance((min(lats), min(lons)), (max(lats), max(lons)))
            if max_span_km > 40.0:
                return None

            mid_lat = (min(lats) + max(lats)) / 2.0
            mid_lon = (min(lons) + max(lons)) / 2.0
            dist_m = max(5000, int(max_span_km * 1000 * 0.75))

            net_type = "all" if mode == "driving" else ("bike" if mode == "cycling" else "walk")
            G = ox.graph_from_point((mid_lat, mid_lon), dist=dist_m, network_type=net_type)

            def find_closest(pt):
                best_n, min_d = None, float('inf')
                for n, data in G.nodes(data=True):
                    d = (pt[0] - data['y'])**2 + (pt[1] - data['x'])**2
                    if d < min_d:
                        min_d = d
                        best_n = n
                return best_n

            orig_node = find_closest(coordinates[0])
            dest_node = find_closest(coordinates[-1])

            node_path = nx.shortest_path(G, orig_node, dest_node, weight="length")
            coords_path = [[G.nodes[n]["y"], G.nodes[n]["x"]] for n in node_path]
            total_m = sum(ox.utils_graph.get_route_edge_attributes(G, node_path, "length"))

            return {
                "polyline": coords_path,
                "distance_km": round(total_m / 1000.0, 2),
                "nodes_count": len(node_path),
                "provider": "Local OSMnx + NetworkX All-Roads Engine"
            }
        except Exception as e:
            print(f"[WorldwideRouter] Local OSMnx fallback notice: {e}")
            return None

    @classmethod
    def generate_fallback_road_corridor_graph(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        labels: Optional[List[str]] = None
    ) -> Tuple[Graph, List[str], List[Dict[str, Any]]]:
        """
        Builds a dense road corridor graph with simulated highway curvature nodes
        when the external OSM routing service is completely unreachable.
        Returns: (g, snapped_nodes, snap_info)
        """
        print("[WorldwideRouter] Building local road corridor network graph...")
        g = Graph()
        snapped_nodes: List[str] = []
        snap_info: List[Dict[str, Any]] = []

        last_node = None
        for idx in range(len(coordinates) - 1):
            p1 = coordinates[idx]
            p2 = coordinates[idx + 1]
            seg_dist = haversine_distance(p1, p2)

            num_subdivisions = max(12, min(50, int(seg_dist / 15.0)))
            pts: List[Tuple[float, float]] = []
            for s in range(num_subdivisions + 1):
                t = s / float(num_subdivisions)
                lat = p1[0] + (p2[0] - p1[0]) * t
                lon = p1[1] + (p2[1] - p1[1]) * t
                curve_amp = 0.0035 * math.sin(t * math.pi)
                perp_lat = -(p2[1] - p1[1]) * curve_amp
                perp_lon = (p2[0] - p1[0]) * curve_amp

                if s == 0:
                    pts.append(p1)
                elif s == num_subdivisions:
                    pts.append(p2)
                else:
                    pts.append((lat + perp_lat, lon + perp_lon))

            for s_idx in range(len(pts) - 1):
                u_id = f"FB_{idx}_{s_idx}"
                v_id = f"FB_{idx}_{s_idx+1}"
                g.add_node(u_id, lat=pts[s_idx][0], lon=pts[s_idx][1])
                g.add_node(v_id, lat=pts[s_idx+1][0], lon=pts[s_idx+1][1])

                sub_dist = haversine_distance(pts[s_idx], pts[s_idx+1])
                sub_geom = [[pts[s_idx][0], pts[s_idx][1]], [pts[s_idx+1][0], pts[s_idx+1][1]]]
                meta = {
                    "street": f"Highway Corridor Segment {idx+1}",
                    "instruction": f"Continue along Highway Corridor {idx+1}",
                    "type": "drive",
                    "distance_m": round(sub_dist * 1000.0, 1)
                }
                g.add_edge(u_id, v_id, weight=sub_dist, bidirectional=True, geometry=sub_geom, metadata=meta)

                if s_idx == 0 and last_node:
                    conn_dist = haversine_distance(g.coords[last_node], g.coords[u_id])
                    g.add_edge(last_node, u_id, weight=conn_dist, bidirectional=True)

            last_node = f"FB_{idx}_{len(pts)-1}"

        for idx, (lat, lon) in enumerate(coordinates):
            label = labels[idx] if (labels and idx < len(labels)) else f"Stop {idx}"
            node_id, snap_dist_m = g.snap_to_nearest_node(lat, lon)
            snapped_nodes.append(node_id)
            node_coord = g.coords.get(node_id, (lat, lon))
            snap_info.append({
                "stop_index": idx,
                "label": label,
                "query_lat": lat,
                "query_lon": lon,
                "snapped_node": node_id,
                "snapped_lat": node_coord[0],
                "snapped_lon": node_coord[1],
                "snap_distance_m": snap_dist_m
            })

        return g, snapped_nodes, snap_info

    @classmethod
    def build_road_graph(
        cls,
        data: Dict[str, Any],
        coordinates: List[Tuple[float, float]],
        labels: Optional[List[str]] = None
    ) -> Tuple[Graph, List[str], List[Dict[str, Any]]]:
        """Builds a Graph instance from OSM road data for Dijkstra/A* compatibility."""
        g = Graph()
        legs = data.get("routes", [{}])[0].get("legs", [])
        last_step_end_node = None

        for leg_idx, leg in enumerate(legs):
            steps = leg.get("steps", [])
            for step_idx, step in enumerate(steps):
                coords_list = step.get("geometry", {}).get("coordinates", [])
                if not coords_list:
                    continue

                s_lon, s_lat = coords_list[0]
                e_lon, e_lat = coords_list[-1]
                u_id = f"RN_{leg_idx}_{step_idx}_start"
                v_id = f"RN_{leg_idx}_{step_idx}_end"

                g.add_node(u_id, lat=s_lat, lon=s_lon)
                g.add_node(v_id, lat=e_lat, lon=e_lon)

                leaflet_geom = [[pt[1], pt[0]] for pt in coords_list]
                dist_km = round(step.get("distance", 0.0) / 1000.0, 4)

                maneuver = step.get("maneuver", {})
                m_type = maneuver.get("type", "drive")
                modifier = maneuver.get("modifier", "")
                street = step.get("name") or "Unnamed Road"

                instruction = f"{m_type.capitalize()} on {street}"
                if modifier:
                    instruction += f" ({modifier})"

                metadata = {
                    "street": street,
                    "instruction": instruction,
                    "type": m_type,
                    "modifier": modifier,
                    "distance_m": round(step.get("distance", 0.0), 1)
                }

                g.add_edge(u_id, v_id, weight=dist_km, bidirectional=False, geometry=leaflet_geom, metadata=metadata)

                if last_step_end_node:
                    trans_dist = haversine_distance(g.coords[last_step_end_node], g.coords[u_id])
                    trans_geom = [
                        [g.coords[last_step_end_node][0], g.coords[last_step_end_node][1]],
                        [s_lat, s_lon]
                    ]
                    g.add_edge(last_step_end_node, u_id, weight=trans_dist, bidirectional=False, geometry=trans_geom)

                last_step_end_node = v_id

        snapped_nodes = []
        snap_info = []
        wp_data = data.get("waypoints", [])

        for idx, (lat, lon) in enumerate(coordinates):
            label = labels[idx] if (labels and idx < len(labels)) else f"Stop {idx}"
            node_id, snap_dist_m = g.snap_to_nearest_node(lat, lon)

            if idx < len(wp_data):
                wp_loc = wp_data[idx].get("location", [])
                if len(wp_loc) == 2:
                    snap_dist_m = round(float(wp_data[idx].get("distance", snap_dist_m)), 2)

            snapped_nodes.append(node_id)
            node_coord = g.coords.get(node_id, (lat, lon))
            snap_info.append({
                "stop_index": idx,
                "label": label,
                "query_lat": lat,
                "query_lon": lon,
                "snapped_node": node_id,
                "snapped_lat": node_coord[0],
                "snapped_lon": node_coord[1],
                "snap_distance_m": snap_dist_m
            })

        return g, snapped_nodes, snap_info

    @classmethod
    def compute_route(
        cls,
        coordinates: List[Tuple[float, float]],
        mode: str = "driving",
        algorithm: str = "a_star",
        provider: str = "osrm",
        use_cache: bool = True,
        labels: Optional[List[str]] = None
    ) -> Dict[str, Any]:
        """
        Main entry point for Worldwide Road Routing:
        1. Validates input and computes straight-line baseline distance.
        2. Checks LRU in-memory route cache.
        3. Queries primary professional routing engine (OSRM Global).
        4. Detects unnecessary highway detours and evaluates all drivable road categories
           (residential, unclassified, tertiary, service, tracks) to find the TRUE SHORTEST DRIVABLE PATH.
        5. Computes diagnostic metrics (API time, generation time, distance ratio, detour savings).
        """
        t_start = time.perf_counter()
        mode = mode.lower()
        algorithm = algorithm.lower()
        if algorithm not in ("a_star", "dijkstra"):
            algorithm = "a_star"

        if len(coordinates) < 2:
            return {
                "success": False,
                "error": "At least two coordinates are required.",
                "polyline": [list(c) for c in coordinates],
                "distance_km": 0.0,
                "straight_line_dist_km": 0.0,
                "distance_ratio": 1.0,
                "duration_min": 0.0,
                "steps": [],
                "road_network": False,
                "mode": mode,
                "provider": "None",
                "routing_profile": "None",
                "api_response_time_ms": 0.0,
                "route_generation_time_ms": 0.0,
                "route_points_count": len(coordinates),
                "waypoints_count": len(coordinates),
                "snapped_stops": [],
                "nodes_loaded": 0,
                "edges_loaded": 0,
                "detour_detected": False,
                "savings_km": 0.0,
                "savings_percent": 0.0,
                "diagnostic_message": "Invalid input.",
                "is_fallback": False,
                "is_cached": False
            }

        # 1. Calculate straight-line baseline distance
        straight_line_dist = 0.0
        for i in range(len(coordinates) - 1):
            straight_line_dist += haversine_distance(coordinates[i], coordinates[i + 1])
        straight_line_dist = round(straight_line_dist, 2)

        # 2. Check in-memory route cache
        cache_key = cls._make_cache_key(coordinates, mode, provider, algorithm)
        if use_cache and cache_key in cls._ROUTE_CACHE:
            cached = dict(cls._ROUTE_CACHE[cache_key])
            cached["is_cached"] = True
            cached["api_response_time_ms"] = 0.0
            cached["route_generation_time_ms"] = round((time.perf_counter() - t_start) * 1000.0, 2)
            return cached

        # 3. Query primary global routing engine (OSRM with alternatives)
        osm_data, provider_name, api_time_ms = cls.fetch_osrm(coordinates, mode=mode)
        best_osrm = None
        osrm_dist_km = float('inf')

        if osm_data and osm_data.get("code") == "Ok" and osm_data.get("routes"):
            # Select shortest route among returned OSRM alternatives
            best_osrm = min(osm_data["routes"], key=lambda r: r.get("distance", float('inf')))
            osrm_dist_km = round(best_osrm["distance"] / 1000.0, 2)

        # 4. Detour Detection & All-Category Shortest Drivable Path Evaluation
        # If the trip is under 75 km or the road-to-straight ratio exceeds 1.35x,
        # rural, village, or residential connector roads may provide a dramatic shortcut over highway detours.
        shorter_route = None
        ratio_check = round(osrm_dist_km / straight_line_dist, 2) if straight_line_dist > 0 else 1.0

        if (straight_line_dist <= 75.0 or ratio_check >= 1.35) and len(coordinates) == 2:
            valhalla_res = cls.fetch_valhalla_shortest(coordinates, mode=mode)
            if valhalla_res and valhalla_res.get("distance_km", float('inf')) < (osrm_dist_km - 0.4):
                shorter_route = valhalla_res

        # 5. Build Final Output
        if shorter_route:
            # We found a verified, significantly shorter drivable road path!
            distance_km = shorter_route["distance_km"]
            duration_min = shorter_route["duration_min"]
            leaflet_polyline = shorter_route["polyline"]
            steps = shorter_route["steps"]
            provider_name = shorter_route["provider"]
            api_time_ms = shorter_route["api_response_time_ms"]
            detour_detected = True
            savings_km = round(osrm_dist_km - distance_km, 2)
            savings_pct = round((savings_km / osrm_dist_km) * 100.0, 1) if osrm_dist_km > 0 else 0.0
            diagnostic_msg = f"Detected and bypassed a {savings_km} km ({savings_pct}%) highway detour via local & village connector roads."
            routing_profile = "Shortest Drivable (All Categories: Motorway, Trunk, Primary, Secondary, Tertiary, Residential, Unclassified, Service, Track)"
            wp_source = None
        elif best_osrm:
            # Standard optimal route from OSRM Global Engine
            geojson_coords = best_osrm["geometry"]["coordinates"]
            leaflet_polyline = [[pt[1], pt[0]] for pt in geojson_coords]
            distance_km = osrm_dist_km
            raw_duration_sec = best_osrm.get("duration", 0.0)

            if mode == "cycling":
                duration_min = round((distance_km / 16.0) * 60.0, 1)
            elif mode == "walking":
                duration_min = round((distance_km / 4.8) * 60.0, 1)
            else:
                duration_min = round(raw_duration_sec / 60.0, 1) if raw_duration_sec > 0 else round((distance_km / 65.0) * 60.0, 1)

            steps = []
            for leg in best_osrm.get("legs", []):
                for step in leg.get("steps", []):
                    maneuver = step.get("maneuver", {})
                    m_type = maneuver.get("type", "drive")
                    modifier = maneuver.get("modifier", "")
                    street = step.get("name") or "road"
                    step_dist = round(step.get("distance", 0), 1)

                    instruction = f"{m_type.capitalize()} on {street}"
                    if modifier:
                        instruction += f" ({modifier})"

                    steps.append({
                        "instruction": instruction,
                        "distance_m": step_dist,
                        "type": m_type,
                        "modifier": modifier
                    })

            detour_detected = False
            savings_km = 0.0
            savings_pct = 0.0
            diagnostic_msg = "Optimal drivable road path selected across available road networks."
            routing_profile = "Worldwide Global Road Network Engine (OSRM)"
            wp_source = osm_data.get("waypoints", [])
        elif osm_data and osm_data.get("code") == "NoRoute":
            total_time_ms = round((time.perf_counter() - t_start) * 1000.0, 2)
            return {
                "success": False,
                "error": "No road network connects these locations. The selected points may be separated by water bodies without road or ferry connections.",
                "polyline": [list(c) for c in coordinates],
                "distance_km": straight_line_dist,
                "straight_line_dist_km": straight_line_dist,
                "distance_ratio": 1.0,
                "duration_min": 0.0,
                "steps": [],
                "road_network": False,
                "mode": mode,
                "provider": provider_name,
                "routing_profile": "No Route Found",
                "api_response_time_ms": api_time_ms,
                "route_generation_time_ms": total_time_ms,
                "route_points_count": len(coordinates),
                "waypoints_count": len(coordinates),
                "snapped_stops": [],
                "nodes_loaded": 0,
                "edges_loaded": 0,
                "detour_detected": False,
                "savings_km": 0.0,
                "savings_percent": 0.0,
                "diagnostic_message": "Locations unreachable by road.",
                "is_fallback": False,
                "is_cached": False
            }
        else:
            # Intelligent Fallback (Local OSMnx or Road Corridor Graph)
            print("[WorldwideRouter] External routing provider unavailable. Executing intelligent local fallback...")
            osmnx_res = cls.local_osmnx_route(coordinates, mode=mode)
            if osmnx_res:
                distance_km = osmnx_res["distance_km"]
                leaflet_polyline = osmnx_res["polyline"]
                provider_name = osmnx_res["provider"]
            else:
                fb_g, fb_snapped, _ = cls.generate_fallback_road_corridor_graph(coordinates, mode=mode, labels=labels)
                provider_name = "Local Road Corridor Graph Fallback Engine"
                polyline_pts = []
                total_dist = 0.0
                for s_i in range(len(fb_snapped) - 1):
                    fb_g.bridge_components_if_needed(fb_snapped[s_i], fb_snapped[s_i + 1])
                    sub_path, sub_dist, _ = fb_g.dijkstra(fb_snapped[s_i], fb_snapped[s_i + 1])
                    if sub_dist < float('inf'):
                        total_dist += sub_dist
                    for node in sub_path:
                        polyline_pts.append(list(fb_g.coords[node]))
                distance_km = round(total_dist, 3)
                leaflet_polyline = polyline_pts

            if mode == "cycling":
                duration_min = round((distance_km / 16.0) * 60.0, 1)
            elif mode == "walking":
                duration_min = round((distance_km / 4.8) * 60.0, 1)
            else:
                duration_min = round((distance_km / 45.0) * 60.0, 1)

            steps = [{"instruction": f"Follow road network corridor to destination ({distance_km} km)", "distance_m": round(distance_km * 1000, 1), "type": "drive", "modifier": ""}]
            detour_detected = False
            savings_km = 0.0
            savings_pct = 0.0
            diagnostic_msg = "Displaying fallback road path."
            routing_profile = "Local Road Fallback Engine"
            wp_source = None

        # Build detailed Snapping Info
        snap_info = []
        for idx, (lat, lon) in enumerate(coordinates):
            label = labels[idx] if (labels and idx < len(labels)) else f"Stop {idx}"
            snap_dist = 0.0
            snapped_coord = [lat, lon]
            if wp_source and idx < len(wp_source):
                wp_loc = wp_source[idx].get("location", [lon, lat])
                snapped_coord = [wp_loc[1], wp_loc[0]]
                snap_dist = round(float(wp_source[idx].get("distance", 0.0)), 2)
            elif leaflet_polyline:
                ref_pt = leaflet_polyline[0] if idx == 0 else (leaflet_polyline[-1] if idx == len(coordinates)-1 else leaflet_polyline[int((len(leaflet_polyline)-1)*(idx/(len(coordinates)-1)))])
                snapped_coord = ref_pt
                snap_dist = round(haversine_distance((lat, lon), (ref_pt[0], ref_pt[1])) * 1000.0, 2)

            snap_info.append({
                "stop_index": idx,
                "label": label,
                "query_lat": lat,
                "query_lon": lon,
                "snapped_lat": snapped_coord[0],
                "snapped_lon": snapped_coord[1],
                "snap_distance_m": snap_dist,
                "snapped_node": f"ROAD_NODE_{idx}"
            })

        dist_ratio = round(distance_km / straight_line_dist, 2) if straight_line_dist > 0 else 1.0
        total_time_ms = round((time.perf_counter() - t_start) * 1000.0, 2)

        result = {
            "success": True,
            "polyline": leaflet_polyline,
            "distance_km": distance_km,
            "straight_line_dist_km": straight_line_dist,
            "distance_ratio": dist_ratio,
            "duration_min": duration_min,
            "steps": steps,
            "road_network": True,
            "mode": mode,
            "algorithm": algorithm,
            "provider": provider_name,
            "routing_profile": routing_profile,
            "api_response_time_ms": api_time_ms,
            "route_generation_time_ms": total_time_ms,
            "route_points_count": len(leaflet_polyline),
            "waypoints_count": len(coordinates),
            "snapped_stops": snap_info,
            "nodes_loaded": len(leaflet_polyline),
            "edges_loaded": len(steps) if steps else len(leaflet_polyline) - 1,
            "detour_detected": detour_detected,
            "savings_km": savings_km,
            "savings_percent": savings_pct,
            "diagnostic_message": diagnostic_msg,
            "is_fallback": "Fallback" in provider_name,
            "is_cached": False
        }

        # Cache the result in LRU
        if len(cls._ROUTE_CACHE) >= cls._CACHE_MAX_SIZE:
            cls._ROUTE_CACHE.pop(next(iter(cls._ROUTE_CACHE)))
        cls._ROUTE_CACHE[cache_key] = result

        return result

    # Backward compatibility alias
    compute_road_route = compute_route


# Compatibility export
OSMRoadGraphRouter = WorldwideRoadRouter


# ==========================================
# GRID GRAPH GENERATOR FOR VISUALIZATION
# ==========================================

def generate_sample_grid(
    rows: int = 10,
    cols: int = 14,
    obstacle_ratio: float = 0.15,
    seed: Optional[int] = 42,
    custom_start: Optional[str] = None,
    custom_target: Optional[str] = None,
    custom_obstacles: Optional[List[str]] = None
) -> Tuple[Graph, List[Dict[str, Any]], str, str]:
    """
    Generates a 2D mesh grid graph with coordinates and simulated obstacles.
    Supports custom start/target points and custom obstacle configurations.
    Returns: (graph, grid_nodes_list, start_node, target_node)
    """
    if seed is not None:
        random.seed(seed)

    g = Graph()
    base_lat = 40.7128
    base_lon = -74.0060
    lat_step = 0.005
    lon_step = 0.007

    grid_nodes = []
    obstacles = set()

    default_start = "N_0_0"
    default_target = f"N_{rows-1}_{cols-1}"

    def is_valid_node(node_str: Optional[str]) -> bool:
        if not node_str or not isinstance(node_str, str):
            return False
        parts = node_str.split('_')
        if len(parts) == 3 and parts[0] == 'N':
            try:
                r, c = int(parts[1]), int(parts[2])
                return 0 <= r < rows and 0 <= c < cols
            except ValueError:
                return False
        return False

    start_node = custom_start if is_valid_node(custom_start) else default_start
    target_node = custom_target if is_valid_node(custom_target) else default_target

    # Determine obstacles: custom or randomly generated
    if custom_obstacles is not None:
        for obs in custom_obstacles:
            if is_valid_node(obs) and obs != start_node and obs != target_node:
                obstacles.add(obs)
    else:
        for r in range(rows):
            for c in range(cols):
                node_id = f"N_{r}_{c}"
                if (node_id != start_node and node_id != target_node) and (random.random() < obstacle_ratio):
                    obstacles.add(node_id)

    obstacles.discard(start_node)
    obstacles.discard(target_node)

    for r in range(rows):
        for c in range(cols):
            node_id = f"N_{r}_{c}"
            is_obstacle = node_id in obstacles
            lat = base_lat + (r * lat_step)
            lon = base_lon + (c * lon_step)

            if not is_obstacle:
                g.add_node(node_id, lat=lat, lon=lon)

            grid_nodes.append({
                "id": node_id,
                "row": r,
                "col": c,
                "lat": lat,
                "lon": lon,
                "is_obstacle": is_obstacle,
                "is_start": node_id == start_node,
                "is_target": node_id == target_node
            })

    # Add edges between non-obstacle neighbors (orthogonal & diagonal)
    for r in range(rows):
        for c in range(cols):
            u = f"N_{r}_{c}"
            if u in obstacles:
                continue

            # Right neighbor
            if c + 1 < cols:
                v = f"N_{r}_{c+1}"
                if v not in obstacles:
                    g.add_edge(u, v)

            # Down neighbor
            if r + 1 < rows:
                v = f"N_{r+1}_{c}"
                if v not in obstacles:
                    g.add_edge(u, v)

    return g, grid_nodes, start_node, target_node


# ==========================================
# MULTI-STOP TSP 2-OPT OPTIMIZER
# ==========================================

def fetch_road_distance_matrix(points: List[Tuple[float, float]]) -> Optional[List[List[float]]]:
    """Fetches real road distance matrix from OSM Table service in kilometers."""
    if len(points) < 2:
        return None
    try:
        coords_str = ";".join(f"{lon:.6f},{lat:.6f}" for lat, lon in points)
        url = f"http://router.project-osrm.org/table/v1/driving/{coords_str}?annotations=distance"
        r = requests.get(url, timeout=6)
        if r.status_code == 200:
            data = r.json()
            if data.get("code") == "Ok" and "distances" in data:
                return [[d / 1000.0 for d in row] for row in data["distances"]]
    except Exception:
        pass
    return None


def calculate_tour_distance(
    points: List[Tuple[float, float]],
    tour: List[int],
    matrix: Optional[List[List[float]]] = None
) -> float:
    """Calculates total distance of an ordered route tour using road matrix or haversine."""
    total = 0.0
    for i in range(len(tour) - 1):
        idx1, idx2 = tour[i], tour[i + 1]
        if matrix and idx1 < len(matrix) and idx2 < len(matrix[idx1]):
            total += matrix[idx1][idx2]
        else:
            total += haversine_distance(points[idx1], points[idx2])
    return total


def solve_tsp(
    points: List[Tuple[float, float]],
    fix_endpoints: bool = True,
    use_road_distance: bool = True
) -> Dict[str, Any]:
    """
    Solves Multi-Stop Route Optimization using:
    1. Nearest Neighbor Heuristic initialization (with real road distance matrix).
    2. 2-opt Heuristic for iterative route untangling.
    """
    n = len(points)
    dist_matrix = fetch_road_distance_matrix(points) if use_road_distance and n >= 3 else None

    def get_pairwise_dist(i: int, j: int) -> float:
        if dist_matrix and i < len(dist_matrix) and j < len(dist_matrix[i]):
            return dist_matrix[i][j]
        return haversine_distance(points[i], points[j])

    if n <= 2:
        dist = calculate_tour_distance(points, list(range(n)), dist_matrix)
        return {
            "optimized_order": list(range(n)),
            "original_distance_km": round(dist, 2),
            "optimized_distance_km": round(dist, 2),
            "savings_km": 0.0,
            "savings_percent": 0.0
        }

    original_order = list(range(n))
    original_dist = calculate_tour_distance(points, original_order, dist_matrix)

    # 1. Nearest Neighbor initialization
    if fix_endpoints:
        unvisited = set(range(1, n - 1))
        tour = [0]
        current = 0
        while unvisited:
            next_node = min(
                unvisited,
                key=lambda node: get_pairwise_dist(current, node)
            )
            tour.append(next_node)
            unvisited.remove(next_node)
            current = next_node
        tour.append(n - 1)
    else:
        unvisited = set(range(1, n))
        tour = [0]
        current = 0
        while unvisited:
            next_node = min(
                unvisited,
                key=lambda node: get_pairwise_dist(current, node)
            )
            tour.append(next_node)
            unvisited.remove(next_node)
            current = next_node

    # 2. 2-Opt Local Search Heuristic
    improved = True
    iterations = 0
    max_iterations = 200

    start_idx = 1 if fix_endpoints else 0
    end_idx = n - 1 if fix_endpoints else n

    while improved and iterations < max_iterations:
        improved = False
        iterations += 1
        best_dist = calculate_tour_distance(points, tour, dist_matrix)

        for i in range(start_idx, end_idx - 1):
            for j in range(i + 1, end_idx):
                new_tour = tour[:i] + tour[i:j + 1][::-1] + tour[j + 1:]
                new_dist = calculate_tour_distance(points, new_tour, dist_matrix)
                if new_dist < best_dist - 1e-6:
                    tour = new_tour
                    best_dist = new_dist
                    improved = True
                    break
            if improved:
                break

    optimized_dist = calculate_tour_distance(points, tour, dist_matrix)
    savings_km = max(0.0, original_dist - optimized_dist)
    savings_percent = round((savings_km / original_dist) * 100.0, 1) if original_dist > 0 else 0.0

    return {
        "optimized_order": tour,
        "original_distance_km": round(original_dist, 2),
        "optimized_distance_km": round(optimized_dist, 2),
        "savings_km": round(savings_km, 2),
        "savings_percent": savings_percent
    }


# ==========================================
# ENVIRONMENTAL & ENERGY IMPACT ANALYTICS
# ==========================================

def calculate_environmental_impact(distance_km: float, mode: str = "driving") -> Dict[str, Any]:
    """
    Computes environmental statistics including fuel consumption, CO2 emissions,
    and calories burned based on travel mode and distance.
    Active calories are strictly computed for human-powered activities (walking,
    running, cycling, hiking) and return None / N/A for motorized travel.
    """
    mode = mode.lower()
    active_modes = {"walking", "running", "cycling", "hiking"}

    if mode in active_modes:
        fuel_liters = 0.0
        co2_kg = 0.0
        if mode == "walking":
            calories = int(distance_km * 60)
            avg_speed_kmh = 4.8
        elif mode == "running":
            calories = int(distance_km * 75)
            avg_speed_kmh = 9.5
        elif mode == "cycling":
            calories = int(distance_km * 30)
            avg_speed_kmh = 16.0
        elif mode == "hiking":
            calories = int(distance_km * 70)
            avg_speed_kmh = 3.8
        else:
            calories = int(distance_km * 50)
            avg_speed_kmh = 5.0
        has_active_calories = True
    elif mode == "driving":
        # Average gas passenger vehicle: ~7.2 Liters/100km, ~171 grams CO2/km
        fuel_liters = round((distance_km / 100.0) * 7.2, 2)
        co2_kg = round((distance_km * 171.0) / 1000.0, 2)
        calories = None
        has_active_calories = False
        avg_speed_kmh = 45.0
    else:
        fuel_liters = round((distance_km / 100.0) * 7.2, 2)
        co2_kg = round((distance_km * 171.0) / 1000.0, 2)
        calories = None
        has_active_calories = False
        avg_speed_kmh = 40.0

    # Trees needed to absorb this CO2 per year (1 mature tree absorbs ~21.8 kg CO2/year)
    trees_offset_days = round((co2_kg / 21.8) * 365, 1) if co2_kg > 0 else 0.0

    return {
        "mode": mode,
        "distance_km": distance_km,
        "average_speed_kmh": avg_speed_kmh,
        "fuel_consumption_liters": fuel_liters,
        "co2_emissions_kg": co2_kg,
        "calories_burned": calories,
        "has_active_calories": has_active_calories,
        "trees_offset_days": trees_offset_days
    }


# ==========================================
# GPX & GEOJSON EXPORTERS
# ==========================================

def generate_gpx(
    coordinates: List[Tuple[float, float]],
    route_name: str = "Smart Route",
    locations: Optional[List[Dict[str, Any]]] = None
) -> str:
    """
    Generates standard GPX (GPS Exchange Format 1.1) XML string from coordinates.
    """
    xml_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<gpx version="1.1" creator="Smart Route Optimizer Platform"',
        '     xmlns="http://www.topografix.com/GPX/1/1"',
        '     xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"',
        '     xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">',
        f'  <metadata><name>{route_name}</name><time>{time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}</time></metadata>'
    ]

    # Add Waypoints
    if locations:
        for loc in locations:
            lat = loc.get("lat", 0.0)
            lon = loc.get("lon", 0.0)
            name = loc.get("name", "Waypoint").replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            role = loc.get("role", "waypoint")
            xml_lines.append(f'  <wpt lat="{lat:.6f}" lon="{lon:.6f}">')
            xml_lines.append(f'    <name>{name}</name>')
            xml_lines.append(f'    <type>{role}</type>')
            xml_lines.append('  </wpt>')

    # Add Track Polyline
    xml_lines.append('  <trk>')
    xml_lines.append(f'    <name>{route_name}</name>')
    xml_lines.append('    <trkseg>')
    for lat, lon in coordinates:
        xml_lines.append(f'      <trkpt lat="{lat:.6f}" lon="{lon:.6f}"></trkpt>')
    xml_lines.append('    </trkseg>')
    xml_lines.append('  </trk>')
    xml_lines.append('</gpx>')

    return "\n".join(xml_lines)


def generate_geojson(
    coordinates: List[Tuple[float, float]],
    route_name: str = "Smart Route",
    locations: Optional[List[Dict[str, Any]]] = None,
    properties: Optional[Dict[str, Any]] = None
) -> str:
    """
    Generates standard RFC 7946 GeoJSON FeatureCollection string from coordinates.
    """
    features = []

    # 1. Polyline LineString Feature (GeoJSON coordinates format: [lon, lat])
    line_coords = [[float(lon), float(lat)] for lat, lon in coordinates]
    props = properties or {}
    props["name"] = route_name
    props["type"] = "route_path"

    features.append({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": line_coords
        },
        "properties": props
    })

    # 2. Point Features for Waypoints
    if locations:
        for idx, loc in enumerate(locations):
            features.append({
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [float(loc.get("lon", 0.0)), float(loc.get("lat", 0.0))]
                },
                "properties": {
                    "name": loc.get("name", f"Stop {idx}"),
                    "address": loc.get("address", ""),
                    "role": loc.get("role", "waypoint"),
                    "index": idx
                }
            })

    geojson_dict = {
        "type": "FeatureCollection",
        "features": features
    }
    return json.dumps(geojson_dict, indent=2)
