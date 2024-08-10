from flask import Flask

def create_app():
    app = Flask(__name__)

    # Import các route từ file routes.py
    from app.routes import init_routes
    init_routes(app)

    return app
