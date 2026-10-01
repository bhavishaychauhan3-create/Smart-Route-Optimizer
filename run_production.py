"""
Production Server Runner & Supervisor for Smart Route Optimizer.
Features:
- Automatic Platform Detection (Gunicorn on POSIX/Linux/Cloud, Waitress on Windows).
- Automatic Crash Detection & Self-Healing Restart Loop.
- Production Thread-Pooling & Concurrency.
- Centralized Production Logging in logs/production_server.log.
"""

import sys
import os
import time
import logging
from pathlib import Path

# Setup logging directory
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_FILE = LOG_DIR / "production_server.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(process)d] %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE, encoding="utf-8"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("production_supervisor")


def run_server_instance(port: int, threads: int):
    """Starts a production server instance depending on OS capability."""
    is_windows = sys.platform.startswith("win")

    if not is_windows:
        # Linux / POSIX / Cloud Containers: Use Gunicorn via subprocess
        import subprocess
        cmd = [
            sys.executable, "-m", "gunicorn",
            "--config", str(BASE_DIR / "gunicorn.conf.py"),
            "app:app"
        ]
        logger.info(f"Starting Gunicorn production server with command: {' '.join(cmd)}")
        return subprocess.call(cmd)
    else:
        # Windows / Local Production: Use Waitress multi-threaded WSGI server
        try:
            import waitress
            from app import app
            logger.info(f"Starting Waitress production WSGI server on 0.0.0.0:{port} with {threads} worker threads")
            waitress.serve(
                app,
                host="0.0.0.0",
                port=port,
                threads=threads,
                channel_timeout=120,
                ident="SmartRouteOptimizer/Production"
            )
            return 0
        except Exception as e:
            logger.error(f"Waitress server encountered critical exception: {e}", exc_info=True)
            return 1


def main():
    port = int(os.environ.get("PORT", 5000))
    threads = int(os.environ.get("PRODUCTION_THREADS", 6))
    max_restarts = int(os.environ.get("MAX_AUTO_RESTARTS", 100))
    restart_count = 0

    logger.info("=======================================================")
    logger.info(" SMART ROUTE OPTIMIZER - PRODUCTION SUPERVISOR ACTIVE")
    logger.info(f" Host: 0.0.0.0 | Port: {port} | Platform: {sys.platform}")
    logger.info(f" Auto-Restart: Enabled | Logfile: {LOG_FILE}")
    logger.info("=======================================================")

    while restart_count < max_restarts:
        start_time = time.time()
        logger.info(f"[Lifecycle] Launching production server instance (Run #{restart_count + 1})...")

        exit_code = run_server_instance(port, threads)
        uptime = time.time() - start_time

        if exit_code == 0:
            logger.info(f"[Lifecycle] Production server shut down cleanly after {uptime:.1f}s.")
            break
        else:
            restart_count += 1
            logger.warning(
                f"[Self-Healing] Production server crashed with exit code {exit_code} after {uptime:.1f}s uptime. "
                f"Auto-restarting in 2 seconds... (Restart #{restart_count}/{max_restarts})"
            )
            time.sleep(2)

    if restart_count >= max_restarts:
        logger.critical(f"Exceeded maximum auto-restart limit ({max_restarts}). Terminating supervisor.")
        sys.exit(1)


if __name__ == "__main__":
    main()
