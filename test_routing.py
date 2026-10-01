"""
tests/test_routing.py - Complete Test Suite for Algorithm Visualizer & Route Optimization Platform
Uses standard library unittest for zero-dependency execution.
Tests:
1. Core Graph & Pathfinding (Dijkstra, A*, Haversine).
2. Stepped Visualization Traces (Dijkstra Stepped, A* Stepped).
3. Side-by-Side Algorithm Comparison Benchmark.
4. Multi-Stop TSP 2-Opt Optimizer.
5. Environmental & Energy Analytics Math.
6. GPX and GeoJSON Export Serializers.
7. Flask Web Pages and REST API Endpoints.
"""

import unittest
import json
import xml.etree.ElementTree as ET
from graph_engine import (
    haversine_distance,
    Graph,
    solve_tsp,
    generate_sample_grid,
    calculate_environmental_impact,
    generate_gpx,
    generate_geojson,
    WorldwideRoadRouter
)
from app import app, fetch_osrm_route


class TestGraphAndVisualizer(unittest.TestCase):

    def test_haversine_known_distance(self):
        # London to Paris (~343 km)
        london = (51.5074, -0.1278)
        paris = (48.8566, 2.3522)
        dist = haversine_distance(london, paris)
        self.assertTrue(340.0 < dist < 350.0)

    def test_dijkstra_stepped_trace(self):
        g, _, start_node, target_node = generate_sample_grid(rows=6, cols=6, obstacle_ratio=0.1, seed=42)
        trace = g.dijkstra_stepped(start_node, target_node)

        self.assertIn("steps", trace)
        self.assertIn("path", trace)
        self.assertTrue(len(trace["steps"]) > 0)
        self.assertTrue(trace["nodes_visited"] > 0)

        # Check action types recorded
        action_types = {step["action"] for step in trace["steps"]}
        self.assertTrue({"current", "visited", "frontier"}.issubset(action_types))

    def test_astar_stepped_trace(self):
        g, _, start_node, target_node = generate_sample_grid(rows=6, cols=6, obstacle_ratio=0.1, seed=42)
        trace = g.a_star_stepped(start_node, target_node)

        self.assertIn("steps", trace)
        self.assertTrue(len(trace["steps"]) > 0)
        # Check that heuristic fields exist in A* steps
        current_steps = [s for s in trace["steps"] if s["action"] == "current"]
        self.assertTrue(len(current_steps) > 0)
        self.assertIn("h", current_steps[0])
        self.assertIn("g", current_steps[0])
        self.assertIn("f", current_steps[0])

    def test_compare_algorithms(self):
        g, _, start_node, target_node = generate_sample_grid(rows=8, cols=8, obstacle_ratio=0.1, seed=99)
        comparison = g.compare_algorithms(start_node, target_node)

        self.assertIn("dijkstra", comparison)
        self.assertIn("a_star", comparison)
        self.assertIn("comparison", comparison)
        self.assertTrue(comparison["dijkstra"]["nodes_explored"] >= comparison["a_star"]["nodes_explored"])
    def test_custom_start_and_target_endpoints(self):
        g, grid_nodes, start_node, target_node = generate_sample_grid(
            rows=8, cols=10, obstacle_ratio=0.1, seed=42,
            custom_start="N_2_3", custom_target="N_5_7"
        )
        self.assertEqual(start_node, "N_2_3")
        self.assertEqual(target_node, "N_5_7")

        # Verify start and target flags in grid_nodes
        start_node_obj = next(n for n in grid_nodes if n["id"] == "N_2_3")
        target_node_obj = next(n for n in grid_nodes if n["id"] == "N_5_7")
        self.assertTrue(start_node_obj["is_start"])
        self.assertFalse(start_node_obj["is_obstacle"])
        self.assertTrue(target_node_obj["is_target"])
        self.assertFalse(target_node_obj["is_obstacle"])

        # Stepped traces should originate at N_2_3 and reach N_5_7
        trace = g.a_star_stepped(start_node, target_node)
        self.assertEqual(trace["path"][0], "N_2_3")
        self.assertEqual(trace["path"][-1], "N_5_7")

    def test_api_visualize_with_custom_endpoints(self):
        app.config["TESTING"] = True
        client = app.test_client()

        payload = {
            "algorithm": "a_star",
            "rows": 8,
            "cols": 10,
            "obstacle_ratio": 0.1,
            "seed": 42,
            "start_node": "N_1_2",
            "target_node": "N_6_8"
        }
        res = client.post('/api/visualize', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["start_node"], "N_1_2")
        self.assertEqual(data["target_node"], "N_6_8")
        self.assertEqual(data["final_path"][0], "N_1_2")
        self.assertEqual(data["final_path"][-1], "N_6_8")


class TestRouteOptimizationAndAnalytics(unittest.TestCase):

    def test_tsp_2opt_optimization(self):
        # 4 coordinates where suboptimal order crosses
        p0 = (0.0, 0.0)
        p1 = (2.0, 8.0)
        p2 = (8.0, 8.0)
        p3 = (10.0, 0.0)

        result = solve_tsp([p0, p2, p1, p3], fix_endpoints=True)
        self.assertLessEqual(result["optimized_distance_km"], result["original_distance_km"])
        self.assertEqual(len(result["optimized_order"]), 4)

    def test_environmental_analytics(self):
        # Test driving stats (100 km)
        drive_stats = calculate_environmental_impact(100.0, mode="driving")
        self.assertAlmostEqual(drive_stats["fuel_consumption_liters"], 7.2, delta=0.1)
        self.assertAlmostEqual(drive_stats["co2_emissions_kg"], 17.1, delta=0.1)
        self.assertEqual(drive_stats["average_speed_kmh"], 45.0)
        self.assertIsNone(drive_stats["calories_burned"])
        self.assertFalse(drive_stats["has_active_calories"])

        # Test cycling stats (20 km)
        bike_stats = calculate_environmental_impact(20.0, mode="cycling")
        self.assertEqual(bike_stats["fuel_consumption_liters"], 0.0)
        self.assertEqual(bike_stats["co2_emissions_kg"], 0.0)
        self.assertTrue(bike_stats["calories_burned"] > 0)
        self.assertTrue(bike_stats["has_active_calories"])
        self.assertEqual(bike_stats["average_speed_kmh"], 16.0)

        # Test walking, running, hiking stats
        walk_stats = calculate_environmental_impact(5.0, mode="walking")
        self.assertTrue(walk_stats["has_active_calories"])
        self.assertEqual(walk_stats["calories_burned"], 300)

        run_stats = calculate_environmental_impact(10.0, mode="running")
        self.assertTrue(run_stats["has_active_calories"])
        self.assertEqual(run_stats["calories_burned"], 750)

        hike_stats = calculate_environmental_impact(10.0, mode="hiking")
        self.assertTrue(hike_stats["has_active_calories"])
        self.assertEqual(hike_stats["calories_burned"], 700)

    def test_transport_mode_duration_scaling(self):
        coords = [(40.7128, -74.0060), (40.7580, -73.9855)]
        car_route = fetch_osrm_route(coords, mode="driving")
        bike_route = fetch_osrm_route(coords, mode="cycling")
        walk_route = fetch_osrm_route(coords, mode="walking")

        self.assertGreater(bike_route["duration_min"], 0)
        self.assertGreater(walk_route["duration_min"], bike_route["duration_min"])


class TestExporters(unittest.TestCase):

    def test_gpx_generation(self):
        coords = [(40.7128, -74.0060), (40.7300, -73.9950), (40.7580, -73.9855)]
        locations = [
            {"name": "Start", "lat": 40.7128, "lon": -74.0060, "role": "start"},
            {"name": "End", "lat": 40.7580, "lon": -73.9855, "role": "destination"}
        ]
        gpx_xml = generate_gpx(coords, route_name="Test Route", locations=locations)

        # Verify valid XML parsing
        root = ET.fromstring(gpx_xml)
        self.assertTrue(root.tag.endswith("gpx"))
        self.assertIn("Test Route", gpx_xml)

    def test_geojson_generation(self):
        coords = [(40.7128, -74.0060), (40.7580, -73.9855)]
        locations = [{"name": "Start", "lat": 40.7128, "lon": -74.0060, "role": "start"}]
        geojson_str = generate_geojson(coords, route_name="Test Route", locations=locations)

        data = json.loads(geojson_str)
        self.assertEqual(data["type"], "FeatureCollection")
        self.assertTrue(len(data["features"]) >= 2)
        self.assertEqual(data["features"][0]["geometry"]["type"], "LineString")


class TestFlaskEndpoints(unittest.TestCase):

    def setUp(self):
        app.config["TESTING"] = True
        self.client = app.test_client()

    def test_pages_render(self):
        # Index
        res1 = self.client.get('/')
        self.assertEqual(res1.status_code, 200)
        self.assertIn(b"Smart Route Optimizer", res1.data)

        # Visualizer
        res2 = self.client.get('/visualizer')
        self.assertEqual(res2.status_code, 200)
        self.assertIn(b"Algorithm Visualizer", res2.data)

    def test_api_visualize(self):
        payload = {"algorithm": "a_star", "rows": 8, "cols": 10, "obstacle_ratio": 0.1}
        res = self.client.post('/api/visualize', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertIn("steps", data)
        self.assertIn("final_path", data)

    def test_api_compare(self):
        payload = {"rows": 8, "cols": 10, "obstacle_ratio": 0.1}
        res = self.client.post('/api/compare', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertIn("dijkstra", data)
        self.assertIn("a_star", data)

    def test_api_autocomplete(self):
        res = self.client.get('/api/autocomplete?q=London')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertIsInstance(data, list)

    def test_api_export_gpx(self):
        payload = {
            "name": "Unit Test Route",
            "polyline": [[40.7128, -74.0060], [40.7580, -73.9855]]
        }
        res = self.client.post('/api/export/gpx', json=payload)
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.mimetype, "application/gpx+xml")
        self.assertIn(b"<gpx", res.data)

    def test_api_export_geojson(self):
        payload = {
            "name": "Unit Test Route",
            "polyline": [[40.7128, -74.0060], [40.7580, -73.9855]]
        }
        res = self.client.post('/api/export/geojson', json=payload)
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.mimetype, "application/geo+json")
        data = json.loads(res.data)
        self.assertEqual(data["type"], "FeatureCollection")


class TestOSMRoadGraphRouting(unittest.TestCase):

    def test_road_graph_node_snapping(self):
        g = Graph()
        g.add_node("N_START", 28.6139, 77.2090)
        g.add_node("N_END", 28.7041, 77.1025)
        g.add_edge("N_START", "N_END", weight=15.0)

        # Snap query slightly offset from N_START
        node_id, snap_dist = g.snap_to_nearest_node(28.6140, 77.2091)
        self.assertEqual(node_id, "N_START")
        self.assertLess(snap_dist, 50.0) # Within 50 meters

    def test_disconnected_component_graceful_bridging(self):
        g = Graph()
        g.add_node("A1", 10.0, 10.0)
        g.add_node("A2", 10.01, 10.01)
        g.add_edge("A1", "A2", weight=1.5)

        g.add_node("B1", 20.0, 20.0)
        g.add_node("B2", 20.01, 20.01)
        g.add_edge("B1", "B2", weight=1.5)

        # Disconnected before bridging
        self.assertFalse(g.is_reachable("A1", "B1"))

        # Bridge gracefully
        bridged = g.bridge_components_if_needed("A1", "B1")
        self.assertTrue(bridged)
        self.assertTrue(g.is_reachable("A1", "B2"))

        path, dist, _ = g.a_star("A1", "B2")
        self.assertEqual(len(path), 4)
        self.assertEqual(path[0], "A1")
        self.assertEqual(path[-1], "B2")

    def test_road_route_generation_with_a_star_and_dijkstra(self):
        coords = [(40.7128, -74.0060), (40.7580, -73.9855)]
        labels = ["Downtown", "Midtown"]

        # Test A*
        route_astar = fetch_osrm_route(coords, mode="driving", algorithm="a_star", labels=labels)
        self.assertTrue(route_astar["success"])
        self.assertTrue(route_astar["road_network"])
        self.assertGreater(route_astar["nodes_loaded"], 0)
        self.assertGreater(route_astar["edges_loaded"], 0)
        self.assertEqual(len(route_astar["snapped_stops"]), 2)
        # Polyline contains real road curvature geometry, not just 2 endpoints
        self.assertGreater(len(route_astar["polyline"]), 10)

        # Test Dijkstra
        route_dijkstra = fetch_osrm_route(coords, mode="driving", algorithm="dijkstra", labels=labels)
        self.assertTrue(route_dijkstra["success"])
        self.assertEqual(route_dijkstra["algorithm"], "dijkstra")
        self.assertGreater(len(route_dijkstra["polyline"]), 10)

    def test_fallback_road_corridor_generation(self):
        from graph_engine import OSMRoadGraphRouter
        coords = [(30.0, 75.0), (31.0, 76.0)]
        g, snapped, info = OSMRoadGraphRouter.generate_fallback_road_corridor_graph(coords)
        self.assertGreater(len(g.adj), 10)
        self.assertEqual(len(snapped), 2)
        self.assertEqual(len(info), 2)

    def test_api_route_endpoint_with_road_diagnostics(self):
        app.config["TESTING"] = True
        client = app.test_client()

        payload = {
            "coordinates": [[40.7128, -74.0060], [40.7580, -73.9855]],
            "mode": "driving",
            "algorithm": "a_star",
            "labels": ["NYC Start", "NYC End"]
        }
        res = client.post('/api/route', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertTrue(data["success"])
        self.assertIn("nodes_loaded", data)
        self.assertIn("edges_loaded", data)
        self.assertIn("snapped_stops", data)
        self.assertGreater(len(data["polyline"]), 10)


class TestWorldwideRouting(unittest.TestCase):

    def test_worldwide_route_between_distant_cities(self):
        coords = [(48.8566, 2.3522), (52.5200, 13.4050)]  # Paris to Berlin (~1050 km)
        route = WorldwideRoadRouter.compute_route(coords, mode="driving")
        self.assertTrue(route["success"])
        self.assertIn("OSRM", route["provider"])
        self.assertGreater(route["distance_km"], 800.0)
        self.assertGreater(route["straight_line_dist_km"], 700.0)
        self.assertGreater(route["distance_ratio"], 1.0)
        self.assertGreater(route["route_points_count"], 1000)
        self.assertGreaterEqual(route["api_response_time_ms"], 0.0)
        self.assertGreaterEqual(route["route_generation_time_ms"], 0.0)
        self.assertFalse(route["is_fallback"])

    def test_in_memory_lru_caching(self):
        WorldwideRoadRouter.clear_cache()
        coords = [(41.8781, -87.6298), (42.3314, -83.0458)]  # Chicago to Detroit
        # Call 1: uncached
        r1 = WorldwideRoadRouter.compute_route(coords, mode="driving")
        self.assertTrue(r1["success"])
        self.assertFalse(r1["is_cached"])

        # Call 2: cached
        r2 = WorldwideRoadRouter.compute_route(coords, mode="driving")
        self.assertTrue(r2["success"])
        self.assertTrue(r2["is_cached"])
        self.assertEqual(r2["api_response_time_ms"], 0.0)
        self.assertLess(r2["route_generation_time_ms"], 20.0)

    def test_straight_line_validation_metric(self):
        coords = [(37.7749, -122.4194), (34.0522, -118.2437)]  # SF to LA
        route = WorldwideRoadRouter.compute_route(coords, mode="driving")
        self.assertTrue(route["success"])
        self.assertTrue(route["distance_km"] > route["straight_line_dist_km"])
        self.assertTrue(1.05 <= route["distance_ratio"] <= 1.5)

    def test_unreachable_road_handling(self):
        coords = [(20.0, -150.0), (0.0, -140.0)]  # Mid-Pacific Ocean
        route = WorldwideRoadRouter.compute_route(coords, mode="driving")
        if not route["success"]:
            self.assertIn("error", route)
        else:
            self.assertTrue(route["is_fallback"])

    def test_worldwide_diagnostics_in_api(self):
        app.config["TESTING"] = True
        client = app.test_client()

        payload = {
            "coordinates": [[51.5074, -0.1278], [53.4808, -2.2426]],  # London to Manchester
            "mode": "driving",
            "algorithm": "a_star",
            "labels": ["London", "Manchester"]
        }
        res = client.post('/api/route', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertTrue(data["success"])
        self.assertIn("provider", data)
        self.assertIn("api_response_time_ms", data)
        self.assertIn("route_generation_time_ms", data)
        self.assertIn("straight_line_dist_km", data)
        self.assertIn("distance_ratio", data)
        self.assertIn("route_points_count", data)
        self.assertIn("waypoints_count", data)
    def test_shortest_drivable_route_tandwal_mullana(self):
        # Origin: MMDU / Tandwal, Destination: Mullana Center
        coords = [(30.233350, 76.982621), (30.275018, 77.048395)]
        route = WorldwideRoadRouter.compute_route(coords, mode="driving")
        self.assertTrue(route["success"])
        # Verifies the 16.05 km highway detour was eliminated and shortest road path found (~10.2 km, matching Google Maps 10.4 km)
        self.assertLessEqual(route["distance_km"], 11.0)
        self.assertGreaterEqual(route["distance_km"], 9.5)
        self.assertTrue(route.get("detour_detected", False))
        self.assertGreater(route.get("savings_km", 0), 3.0)
        self.assertGreater(route.get("savings_percent", 0), 25.0)

    def test_api_route_includes_detour_diagnostics(self):
        app.config["TESTING"] = True
        client = app.test_client()

        payload = {
            "coordinates": [[30.233350, 76.982621], [30.275018, 77.048395]],
            "mode": "driving",
            "algorithm": "a_star",
            "labels": ["Tandwal", "Mullana"]
        }
        res = client.post('/api/route', json=payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertTrue(data["success"])
        self.assertLessEqual(data["distance_km"], 11.0)
        self.assertTrue(data.get("detour_detected", False))
        self.assertIn("savings_km", data)
        self.assertIn("diagnostic_message", data)
        self.assertIn("Shortest Drivable", data.get("routing_profile", ""))


if __name__ == '__main__':
    unittest.main()


