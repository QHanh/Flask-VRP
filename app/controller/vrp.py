from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import numpy as np

def create_data_model(input):
    url = "https://api.geoapify.com/v1/routematrix?apiKey=6f9407cd46194768808f06a0fe952550"

    headers = {"Content-Type": "application/json"}

    # read json file
    json_data = input

    # data input
    if 'time_windows' in json_data:
        time_windows = [(time[0], time[1]) for time_obj in json_data['time_windows'] for time in [time_obj["time"]]]
    demands = json_data['demands']
    num_vehicles = json_data['num_vehicles']
    vehicle_capacities = json_data['vehicle_capacities']
    starts = json_data['starts']
    ends = json_data['ends']

    # data coordinate request
    post = {
        'mode': 'drive', 
        'sources': json_data['coordinates'], 
        'targets': json_data['coordinates']
    }

    # request
    resp = requests.post(url, headers=headers, json=post)

    # Raise an HTTPError for bad responses
    resp.raise_for_status()

    # If the request is successful, extract data from the response
    result_data = resp.json()

    num_sources = len(result_data["sources"])
    num_targets = len(result_data["targets"])

    # Initialize distance and time matrices with large values
    distance_matrix = np.zeros((num_sources, num_targets)).astype(int)
    time_matrix = np.zeros((num_sources, num_targets)).astype(int)

    # Populate matrices with actual values
    for result in result_data["sources_to_targets"]:
        for item in result:
            source_index = item["source_index"]
            target_index = item["target_index"]
            distance_matrix[source_index][target_index] = item["distance"]
            time_matrix[source_index][target_index] = item["time"]
    
    data = {}
    data["distance_matrix"] = distance_matrix
    data["demands"] = demands
    data["num_vehicles"] = num_vehicles
    data["vehicle_capacities"] = vehicle_capacities
    data["starts"] = starts
    data["ends"] = ends
    assert data['num_vehicles'] == len(data['vehicle_capacities'])
    assert data['num_vehicles'] == len(data['starts'])
    assert data['num_vehicles'] == len(data['ends'])
    return data

def print_solution(data, manager, routing, solution):
    """Chuyển đổi hàm print_solution để trả về chuỗi văn bản."""
    result = []

    # Objective value
    result.append(f"Objective: {solution.ObjectiveValue()}")

    # Display dropped nodes
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += f' {manager.IndexToNode(node)}'
    result.append(dropped_nodes)

    # Print routes
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            plan_output += f" {node_index} Load({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f" {manager.IndexToNode(index)} Load({route_load})\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        plan_output += f"Load of the route: {route_load}\n"
        result.append(plan_output)
        total_distance += route_distance
        total_load += route_load

    result.append(f"Total distance of all routes: {total_distance}m")
    result.append(f"Total load of all routes: {total_load}")

    # Kết hợp tất cả thông tin vào một chuỗi văn bản
    return "\n".join(result)


def main(input):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(input)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["starts"], data["ends"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Allow to drop nodes.
    penalty = 1000000
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    result_str = print_solution(data, manager, routing, solution)
    
    return result_str
