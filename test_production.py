import sys
import urllib.request
import json
import time

def run_tests(base_url="https://bin-matching-reggae-joan.trycloudflare.com"):
    print(f"--- RUNNING PRODUCTION ENDPOINT VERIFICATION: {base_url} ---")
    results = {}

    # 1. Healthcheck Probe
    try:
        resp = urllib.request.urlopen(f"{base_url}/health", timeout=10)
        data = json.loads(resp.read().decode('utf-8'))
        assert resp.status == 200
        assert data.get("status") == "healthy"
        results["Healthcheck"] = f"PASS (Status 200, uptime: {data.get('uptime_seconds')}s)"
    except Exception as e:
        results["Healthcheck"] = f"FAIL: {e}"

    # 2. Homepage Delivery
    try:
        resp = urllib.request.urlopen(f"{base_url}/", timeout=10)
        html = resp.read().decode('utf-8')
        assert resp.status == 200
        assert "Smart Route Optimizer" in html
        assert "hero-bg-container" in html
        results["Homepage UI"] = f"PASS (Status 200, length: {len(html)} bytes)"
    except Exception as e:
        results["Homepage UI"] = f"FAIL: {e}"

    # 3. Static Asset Caching
    try:
        resp = urllib.request.urlopen(f"{base_url}/static/images/hero_map_bg.jpg", timeout=10)
        cache_hdr = resp.headers.get("Cache-Control", "")
        cl = resp.headers.get("Content-Length", 0)
        assert resp.status == 200
        assert "max-age" in cache_hdr
        results["Static Asset Caching"] = f"PASS (Status 200, Cache-Control: {cache_hdr}, Size: {cl} bytes)"
    except Exception as e:
        results["Static Asset Caching"] = f"FAIL: {e}"

    # 4. Guide Educational Page
    try:
        resp = urllib.request.urlopen(f"{base_url}/guide", timeout=10)
        html = resp.read().decode('utf-8')
        assert resp.status == 200
        assert "Dijkstra" in html
        results["Educational Guide Page"] = f"PASS (Status 200, length: {len(html)} bytes)"
    except Exception as e:
        results["Educational Guide Page"] = f"FAIL: {e}"

    # 5. Visualizer Lab Page
    try:
        resp = urllib.request.urlopen(f"{base_url}/visualizer", timeout=10)
        html = resp.read().decode('utf-8')
        assert resp.status == 200
        assert "Visualizer" in html
        results["Visualizer Lab Page"] = f"PASS (Status 200, length: {len(html)} bytes)"
    except Exception as e:
        results["Visualizer Lab Page"] = f"FAIL: {e}"

    # 6. Algorithm API - A* Search
    try:
        payload = json.dumps({"algorithm": "a_star", "heuristic": "manhattan", "grid_size": 6, "allow_diagonal": False}).encode("utf-8")
        req = urllib.request.Request(f"{base_url}/api/visualize", data=payload, headers={"Content-Type": "application/json"})
        resp = urllib.request.urlopen(req, timeout=10)
        data = json.loads(resp.read().decode("utf-8"))
        assert resp.status == 200
        assert "final_path" in data
        assert len(data["final_path"]) > 0
        results["A* Algorithm API"] = f"PASS (Visited: {data.get('nodes_visited')}, Path steps: {len(data['final_path'])})"
    except Exception as e:
        results["A* Algorithm API"] = f"FAIL: {e}"

    # 7. Algorithm API - Dijkstra
    try:
        payload = json.dumps({"algorithm": "dijkstra", "grid_size": 6, "allow_diagonal": False}).encode("utf-8")
        req = urllib.request.Request(f"{base_url}/api/visualize", data=payload, headers={"Content-Type": "application/json"})
        resp = urllib.request.urlopen(req, timeout=10)
        data = json.loads(resp.read().decode("utf-8"))
        assert resp.status == 200
        assert "final_path" in data
        assert len(data["final_path"]) > 0
        results["Dijkstra Algorithm API"] = f"PASS (Visited: {data.get('nodes_visited')}, Path steps: {len(data['final_path'])})"
    except Exception as e:
        results["Dijkstra Algorithm API"] = f"FAIL: {e}"

    # 8. Geocoding Autocomplete API
    try:
        resp = urllib.request.urlopen(f"{base_url}/api/autocomplete?q=London", timeout=10)
        data = json.loads(resp.read().decode("utf-8"))
        assert resp.status == 200
        assert isinstance(data, list)
        results["Geocoding Autocomplete API"] = f"PASS (Results returned: {len(data)})"
    except Exception as e:
        results["Geocoding Autocomplete API"] = f"FAIL: {e}"

    # Print Report
    print("\n================== DEPLOYMENT TEST REPORT ==================")
    all_passed = True
    for test_name, res in results.items():
        print(f"[{'OK' if res.startswith('PASS') else 'FAIL'}] {test_name}: {res}")
        if not res.startswith("PASS"):
            all_passed = False
    print("============================================================\n")
    return all_passed

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
