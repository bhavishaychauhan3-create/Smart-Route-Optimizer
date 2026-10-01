"""
WSGI Entry Point for Production Deployment.
Exposes the Flask 'app' callable for WSGI servers like Gunicorn and Waitress.
"""

from app import app

if __name__ == "__main__":
    app.run()
