import sys
import time
import os
from pathlib import Path
from pycloudflared import try_cloudflare

LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_DIR.mkdir(exist_ok=True)
TUNNEL_LOG = LOG_DIR / "tunnel.log"

port = int(os.environ.get("PORT", 5000))
print(f"Initiating Cloudflare Production Tunnel for port {port}...")

try:
    tunnel_url = try_cloudflare(port=port)
    print(f"=====================================================")
    print(f" PUBLIC HTTPS PRODUCTION URL: {tunnel_url}")
    print(f"=====================================================")
    with open(TUNNEL_LOG, "w", encoding="utf-8") as f:
        f.write(f"{tunnel_url}\n")
    
    # Keep the tunnel process running indefinitely
    while True:
        time.sleep(60)
except Exception as e:
    print(f"Error starting Cloudflare tunnel: {e}")
    sys.exit(1)
