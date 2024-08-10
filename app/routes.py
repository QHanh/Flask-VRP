from flask import request, jsonify, Response
from app.controller.vrp import main

def init_routes(app):
    @app.route('/', methods=['POST'])
    def solve():
        input_data = request.json
        solution = main(input_data)
        return Response(solution, mimetype='text/plain')