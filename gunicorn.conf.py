"""
Production Gunicorn Configuration for Smart Route Optimizer.
Optimized for high-concurrency, routing calculation workers, and memory stability.
"""

import os
import multiprocessing

# Port and binding
port = os.environ.get("PORT", "5000")
bind = f"0.0.0.0:{port}"

# Worker processes and threads
# Compute workers based on CPU count, with a safe ceiling for container environments
default_workers = min(multiprocessing.cpu_count() * 2 + 1, 4)
workers = int(os.environ.get("WEB_CONCURRENCY", default_workers))
threads = int(os.environ.get("PYTHON_GET_THREADS", 2))
worker_class = "gthread"

# Request timeouts - 120s allows long-distance multi-stop TSP / OSM calculations to finish
timeout = int(os.environ.get("GUNICORN_TIMEOUT", 120))
keepalive = 5

# Memory management & recycling to avoid memory leaks
max_requests = 1000
max_requests_jitter = 100

# Logging
accesslog = "-"
errorlog = "-"
loglevel = os.environ.get("LOG_LEVEL", "info")
access_log_format = '%(h)s %(l)s %(u)s %(t)s "%(r)s" %(s)s %(b)s "%(f)s" "%(a)s" (%(D)s µs)'

# Process naming
proc_name = "smart-route-optimizer-gunicorn"
preload_app = False
