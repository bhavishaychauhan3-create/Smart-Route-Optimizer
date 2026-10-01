"""
app.py - Navigation System & Algorithm Visualization Platform
Flask backend supporting:
1. Multi-modal Road Routing (Driving, Cycling, Walking) via OSRM.
2. Defensive Geocoding & Reverse Geocoding with OSM Nominatim.
3. Intelligent Address Autocomplete via Photon & Nominatim.
4. Algorithm Visualizer & Comparison Engine (Dijkstra vs A*).
5. Multi-Stop TSP 2-Opt Tour Optimization.
6. Route Analytics (Fuel, Carbon Footprint, Efficiency).
7. Export Capabilities (GPX and GeoJSON Downloads).
"""

import sys
import os
import time
from typing import List, Tuple, Dict, Any, Optional
import requests
from flask import Flask, request, render_template, jsonify, Response, redirect
from geopy.geocoders import Nominatim
from geopy.exc import GeocoderTimedOut, GeocoderServiceError

from graph_engine import (
    haversine_distance,
    solve_tsp,
    Graph,
    generate_sample_grid,
    calculate_environmental_impact,
    generate_gpx,
    generate_geojson,
    OSMRoadGraphRouter,
    WorldwideRoadRouter
)

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'smart-navigation-secret-key-prod-9831')
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = int(os.environ.get('STATIC_CACHE_SECONDS', 31536000))
SERVER_START_TIME = time.time()


@app.after_request
def set_production_headers(response):
    """Enforces high-performance caching for static assets in production."""
    if request.path.startswith('/static/'):
        response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
    return response


@app.route('/health', methods=['GET'])
def health_check():
    """Production healthcheck probe for monitoring and auto-restart supervisors."""
    return jsonify({
        "status": "healthy",
        "service": "Smart Route Optimizer",
        "timestamp": int(time.time()),
        "uptime_seconds": round(time.time() - SERVER_START_TIME, 2),
        "environment": os.environ.get("FLASK_ENV", "production")
    }), 200


# Compliant user agent for Nominatim
GEOLOCATOR = Nominatim(user_agent="smart_navigation_route_optimizer_v2_academic")
OSRM_BASE = "http://router.project-osrm.org/route/v1"
PHOTON_API = "https://photon.komoot.io/api"


def safe_geocode(query: str, max_retries: int = 2) -> Optional[Any]:
    """Safely resolves address to coordinates with retries and timeout handling."""
    if not query or not query.strip():
        return None
    for attempt in range(max_retries):
        try:
            loc = GEOLOCATOR.geocode(query.strip(), timeout=4)
            if loc:
                return loc
            return None
        except (GeocoderTimedOut, GeocoderServiceError):
            time.sleep(0.4)
            if attempt == max_retries - 1:
                return None
        except Exception:
            return None
    return None


def safe_reverse(lat: float, lon: float) -> str:
    """Safely reverse-geocodes coordinate to address string."""
    try:
        loc = GEOLOCATOR.reverse((lat, lon), timeout=4)
        if loc and loc.address:
            return loc.address
    except Exception:
        pass
    return f"Coordinates: {lat:.4f}, {lon:.4f}"


def fetch_osrm_route(
    coordinates: List[Tuple[float, float]],
    mode: str = "driving",
    algorithm: str = "a_star",
    labels: Optional[List[str]] = None
) -> Dict[str, Any]:
    """
    Computes worldwide road-network routing using professional routing services
    (OSRM Global Cluster, OpenRouteService), with intelligent local fallback,
    in-memory caching, straight-line distance validation, and comprehensive diagnostics.
    """
    return WorldwideRoadRouter.compute_route(
        coordinates,
        mode=mode,
        algorithm=algorithm,
        labels=labels
    )


# ==========================================
# WEB PAGE ROUTES
# ==========================================

@app.route('/')
@app.route('/planner')
@app.route('/route-planner')
def index():
    """Renders main search dashboard and route planner."""
    return render_template('index.html')


@app.route('/visualizer')
def visualizer_page():
    """Renders the Algorithm Visualization & Comparison Lab."""
    return render_template('visualizer.html')


@app.route('/guide')
@app.route('/learn')
@app.route('/educational')
@app.route('/comparison')
def guide_page():
    """Renders the comprehensive A* vs Dijkstra Educational & Visualizer Web Application."""
    return render_template('guide.html')


@app.route('/map', methods=['GET', 'POST'])
def map_page():
    """
    Main route generation endpoint with multi-modal support and eco analytics.
    """
    if request.method == 'GET':
        return redirect('/#route-planner')
    raw_lat = request.form.get('latitude', '').strip()
    raw_lon = request.form.get('longitude', '').strip()
    source_query = request.form.get('source', '').strip()
    destination_query = request.form.get('destination', '').strip()
    should_optimize = request.form.get('optimize', 'false').lower() == 'true'
    transport_mode = request.form.get('mode', 'driving').lower()
    routing_algo = request.form.get('algorithm', 'a_star').lower()

    # Collect additional waypoints/stops if provided
    raw_stops = request.form.getlist('stops[]')
    stops_queries = [s.strip() for s in raw_stops if s.strip()]

    if not destination_query:
        return render_template('index.html', error="Destination is required to generate a route.")

    # 1. Resolve Starting Point
    if source_query:
        source_loc = safe_geocode(source_query)
        if not source_loc:
            return render_template('index.html', error=f"Could not locate starting point: '{source_query}'.")
        start_lat, start_lon = source_loc.latitude, source_loc.longitude
        source_label = source_query
        source_address = source_loc.address
    else:
        if not raw_lat or not raw_lon:
            return render_template(
                'index.html',
                error="Could not detect browser GPS. Please enter a Starting Point manually."
            )
        try:
            start_lat, start_lon = float(raw_lat), float(raw_lon)
        except ValueError:
            return render_template('index.html', error="Invalid GPS coordinates received.")

        source_label = "My Location"
        source_address = safe_reverse(start_lat, start_lon)

    # 2. Resolve Destination Point
    dest_loc = safe_geocode(destination_query)
    if not dest_loc:
        return render_template('index.html', error=f"Could not locate destination: '{destination_query}'.")
    dest_lat, dest_lon = dest_loc.latitude, dest_loc.longitude
    dest_label = destination_query
    dest_address = dest_loc.address

    # 3. Assemble Waypoints
    route_points: List[Tuple[float, float]] = [(start_lat, start_lon)]
    location_cards: List[Dict[str, Any]] = [{
        "name": source_label,
        "address": source_address,
        "lat": start_lat,
        "lon": start_lon,
        "role": "start"
    }]

    for idx, stop_text in enumerate(stops_queries):
        stop_loc = safe_geocode(stop_text)
        if stop_loc:
            route_points.append((stop_loc.latitude, stop_loc.longitude))
            location_cards.append({
                "name": stop_text,
                "address": stop_loc.address,
                "lat": stop_loc.latitude,
                "lon": stop_loc.longitude,
                "role": "waypoint"
            })

    route_points.append((dest_lat, dest_lon))
    location_cards.append({
        "name": dest_label,
        "address": dest_address,
        "lat": dest_lat,
        "lon": dest_lon,
        "role": "destination"
    })

    # 4. Multi-Stop Route Optimization (TSP 2-opt)
    tsp_stats: Optional[Dict[str, Any]] = None
    if should_optimize and len(route_points) > 3:
        tsp_result = solve_tsp(route_points, fix_endpoints=True, use_road_distance=True)
        new_order = tsp_result["optimized_order"]
        route_points = [route_points[i] for i in new_order]
        location_cards = [location_cards[i] for i in new_order]
        tsp_stats = {
            "original_distance_km": tsp_result["original_distance_km"],
            "optimized_distance_km": tsp_result["optimized_distance_km"],
            "savings_km": tsp_result["savings_km"],
            "savings_percent": tsp_result["savings_percent"]
        }

    # 5. Fetch Road Network Routing using OpenStreetMap Road Graph and Dijkstra/A*
    stop_labels = [card["name"] for card in location_cards]
    routing_data = fetch_osrm_route(
        route_points,
        mode=transport_mode,
        algorithm=routing_algo,
        labels=stop_labels
    )

    # 6. Environmental Analytics (Fuel, CO2)
    eco_stats = calculate_environmental_impact(routing_data["distance_km"], mode=transport_mode)

    return render_template(
        'map.html',
        source=source_label,
        source_address=source_address,
        destination=dest_label,
        dest_address=dest_address,
        latitude=start_lat,
        longitude=start_lon,
        dest_lat=dest_lat,
        dest_lon=dest_lon,
        distance=routing_data["distance_km"],
        straight_line_dist_km=routing_data.get("straight_line_dist_km", routing_data["distance_km"]),
        distance_ratio=routing_data.get("distance_ratio", 1.0),
        duration=routing_data["duration_min"],
        route_polyline=routing_data["polyline"],
        steps=routing_data["steps"],
        is_road_network=routing_data["road_network"],
        mode=transport_mode,
        algorithm=routing_data.get("algorithm", routing_algo),
        provider=routing_data.get("provider", "OSRM Global Road Network Engine"),
        routing_profile=routing_data.get("routing_profile", "Shortest Drivable (All Categories)"),
        api_response_time_ms=routing_data.get("api_response_time_ms", 0.0),
        route_generation_time_ms=routing_data.get("route_generation_time_ms", 0.0),
        route_points_count=routing_data.get("route_points_count", len(routing_data.get("polyline", []))),
        waypoints_count=routing_data.get("waypoints_count", len(route_points)),
        is_fallback=routing_data.get("is_fallback", False),
        fallback_warning=routing_data.get("fallback_warning", ""),
        is_cached=routing_data.get("is_cached", False),
        nodes_loaded=routing_data.get("nodes_loaded", 0),
        edges_loaded=routing_data.get("edges_loaded", 0),
        snapped_stops=routing_data.get("snapped_stops", []),
        detour_detected=routing_data.get("detour_detected", False),
        savings_km=routing_data.get("savings_km", 0.0),
        savings_percent=routing_data.get("savings_percent", 0.0),
        diagnostic_message=routing_data.get("diagnostic_message", ""),
        locations=location_cards,
        tsp_stats=tsp_stats,
        eco_stats=eco_stats
    )


# ==========================================
# REST API ENDPOINTS
# ==========================================

@app.route('/api/route', methods=['POST'])
def api_route():
    """
    JSON API endpoint for computing routes between multiple coordinate points.
    Payload: {"coordinates": [[lat1, lon1], [lat2, lon2]], "mode": "driving", "algorithm": "a_star", "labels": [...]}
    """
    data = request.get_json(silent=True) or {}
    coords = data.get("coordinates", [])
    mode = data.get("mode", "driving")
    algo = data.get("algorithm", "a_star")
    labels = data.get("labels")

    if not isinstance(coords, list) or len(coords) < 2:
        return jsonify({"error": "At least two [lat, lon] coordinate pairs are required."}), 400

    try:
        parsed_coords = [(float(c[0]), float(c[1])) for c in coords]
    except (ValueError, IndexError):
        return jsonify({"error": "Invalid coordinate formatting."}), 400

    route_info = fetch_osrm_route(parsed_coords, mode=mode, algorithm=algo, labels=labels)
    return jsonify(route_info)


@app.route('/api/optimize', methods=['POST'])
def api_optimize():
    """
    JSON API endpoint for TSP Multi-Stop Optimization.
    Payload: {"coordinates": [[lat1, lon1], ...], "fix_endpoints": true}
    """
    data = request.get_json(silent=True) or {}
    coords = data.get("coordinates", [])
    fix_endpoints = data.get("fix_endpoints", True)

    if not isinstance(coords, list) or len(coords) < 3:
        return jsonify({"error": "Optimization requires at least 3 points."}), 400

    try:
        parsed_coords = [(float(c[0]), float(c[1])) for c in coords]
    except (ValueError, IndexError):
        return jsonify({"error": "Invalid coordinate format."}), 400

    tsp_res = solve_tsp(parsed_coords, fix_endpoints=fix_endpoints, use_road_distance=True)
    return jsonify(tsp_res)


@app.route('/api/autocomplete', methods=['GET'])
def api_autocomplete():
    """
    Instant search suggestions using Photon API (OpenStreetMap geocoding engine).
    Query: /api/autocomplete?q=New+York
    """
    query = request.args.get('q', '').strip()
    if len(query) < 2:
        return jsonify([])

    try:
        resp = requests.get(f"{PHOTON_API}?q={query}&limit=6", timeout=3)
        if resp.status_code == 200:
            data = resp.json()
            results = []
            for item in data.get("features", []):
                props = item.get("properties", {})
                coords = item.get("geometry", {}).get("coordinates", [0, 0])
                name = props.get("name", "")
                city = props.get("city") or props.get("state") or props.get("country", "")
                full_desc = f"{name}, {city}".strip(", ") if city else name

                results.append({
                    "name": name,
                    "description": full_desc,
                    "city": props.get("city", ""),
                    "country": props.get("country", ""),
                    "lat": coords[1],
                    "lon": coords[0]
                })
            return jsonify(results)
    except Exception as e:
        print(f"[Notice] Photon autocomplete query fallback: {e}")

    # Fallback to local geocode query
    loc = safe_geocode(query)
    if loc:
        return jsonify([{
            "name": query,
            "description": loc.address,
            "lat": loc.latitude,
            "lon": loc.longitude
        }])
    return jsonify([])


@app.route('/api/visualize', methods=['POST'])
def api_visualize():
    """
    Generates step-by-step exploration trace for algorithm visualization.
    Payload: {"algorithm": "dijkstra"|"a_star", "rows": 10, "cols": 14, "obstacle_ratio": 0.15, "seed": 42, "start_node": "N_0_0", "target_node": "N_9_13", "obstacles": [...]}
    """
    data = request.get_json(silent=True) or {}
    algo = data.get("algorithm", "dijkstra").lower()
    rows = max(5, min(20, int(data.get("rows", 10))))
    cols = max(5, min(25, int(data.get("cols", 14))))
    obstacle_ratio = max(0.0, min(0.4, float(data.get("obstacle_ratio", 0.15))))
    seed = data.get("seed", 42)
    start_node = data.get("start_node")
    target_node = data.get("target_node")
    obstacles = data.get("obstacles")

    g, grid_nodes, start_node, target_node = generate_sample_grid(
        rows=rows, cols=cols, obstacle_ratio=obstacle_ratio, seed=seed,
        custom_start=start_node, custom_target=target_node, custom_obstacles=obstacles
    )

    t0 = time.perf_counter_ns()
    if algo == "a_star":
        trace = g.a_star_stepped(start_node, target_node)
    else:
        trace = g.dijkstra_stepped(start_node, target_node)
    exec_time_us = round((time.perf_counter_ns() - t0) / 1000.0, 2)

    return jsonify({
        "algorithm": algo,
        "grid_nodes": grid_nodes,
        "start_node": start_node,
        "target_node": target_node,
        "rows": rows,
        "cols": cols,
        "steps": trace["steps"],
        "final_path": trace["path"],
        "distance": trace["distance"],
        "nodes_visited": trace["nodes_visited"],
        "execution_time_us": exec_time_us
    })


@app.route('/api/compare', methods=['POST'])
def api_compare():
    """
    Side-by-side benchmark comparison between Dijkstra and A*.
    Payload: {"rows": 12, "cols": 16, "obstacle_ratio": 0.15, "seed": 42, "start_node": "N_0_0", "target_node": "N_11_15", "obstacles": [...]}
    """
    data = request.get_json(silent=True) or {}
    rows = max(5, min(20, int(data.get("rows", 12))))
    cols = max(5, min(25, int(data.get("cols", 16))))
    obstacle_ratio = max(0.0, min(0.4, float(data.get("obstacle_ratio", 0.15))))
    seed = data.get("seed", 42)
    start_node = data.get("start_node")
    target_node = data.get("target_node")
    obstacles = data.get("obstacles")

    g, grid_nodes, start_node, target_node = generate_sample_grid(
        rows=rows, cols=cols, obstacle_ratio=obstacle_ratio, seed=seed,
        custom_start=start_node, custom_target=target_node, custom_obstacles=obstacles
    )

    comparison = g.compare_algorithms(start_node, target_node)
    comparison["grid_info"] = {
        "rows": rows,
        "cols": cols,
        "total_nodes": len(grid_nodes),
        "active_graph_nodes": len(g.adj)
    }
    return jsonify(comparison)


@app.route('/api/benchmark', methods=['GET'])
def api_benchmark():
    """Preserved benchmark endpoint comparing Dijkstra vs A*."""
    g, _, start_node, target_node = generate_sample_grid(rows=8, cols=8, obstacle_ratio=0.1, seed=10)
    comparison = g.compare_algorithms(start_node, target_node)
    return jsonify(comparison)


# ==========================================
# EXPORT ENDPOINTS (GPX & GEOJSON)
# ==========================================

@app.route('/api/export/gpx', methods=['POST'])
def export_gpx():
    """Exports route polyline as standard .gpx GPS XML."""
    data = request.get_json(silent=True) or {}
    polyline = data.get("polyline", [])
    locations = data.get("locations", [])
    route_name = data.get("name", "Smart Navigation Route")

    if not polyline:
        return jsonify({"error": "No polyline points to export."}), 400

    gpx_xml = generate_gpx(polyline, route_name=route_name, locations=locations)
    return Response(
        gpx_xml,
        mimetype="application/gpx+xml",
        headers={"Content-Disposition": f"attachment; filename=route_{int(time.time())}.gpx"}
    )


@app.route('/api/export/geojson', methods=['POST'])
def export_geojson():
    """Exports route polyline and waypoints as standard RFC 7946 .geojson."""
    data = request.get_json(silent=True) or {}
    polyline = data.get("polyline", [])
    locations = data.get("locations", [])
    route_name = data.get("name", "Smart Navigation Route")
    props = {
        "distance_km": data.get("distance_km", 0.0),
        "duration_min": data.get("duration_min", 0.0),
        "mode": data.get("mode", "driving")
    }
    

    if not polyline:
        return jsonify({"error": "No polyline points to export."}), 400

    geojson_str = generate_geojson(polyline, route_name=route_name, locations=locations, properties=props)
    return Response(
        geojson_str,
        mimetype="application/geo+json",
        headers={"Content-Disposition": f"attachment; filename=route_{int(time.time())}.geojson"}
    )


if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))
    debug_mode = os.environ.get("FLASK_DEBUG", "false").lower() in ("true", "1", "yes")
    app.run(host="0.0.0.0", port=port, debug=debug_mode)