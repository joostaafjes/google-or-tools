"""Capacited Vehicles Routing Problem (CVRP)."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from timeit import default_timer as timer

import numpy as np
import scipy.spatial.distance as ssd

import plotly.graph_objects as go


def calculate_distances(X_Coordinations, Y_Coordinations):
    # Stack them on top of each other
    X_Y_Coordinations = np.column_stack((X_Coordinations, Y_Coordinations))

    # Create a distance matrix from the Euclidean distance
    Distance_Matrix = ssd.cdist(X_Y_Coordinations, X_Y_Coordinations, 'euclidean')

    dist_matrix = Distance_Matrix.astype(int).tolist()
    return dist_matrix


def create_data_model(X_Coordinations, Y_Coordinations):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = calculate_distances(X_Coordinations, Y_Coordinations)
    data['demands'] = [0, 19, 21, 6, 19, 7, 12, 16, 6, 16, 8, 14, 21, 16, 3, 22, 18, 19, 1, 24, 8, 12, 4, 8, 24, 24, 2, 20, 15, 2, 14, 9]
    data['vehicle_capacities'] = [100, 100, 100, 100, 100]
    data['num_vehicles'] = 5
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))

def plot_solution(data, manager, routing, solution, X_Coordinations, Y_Coordinations):
    """Plot solution."""
    fig = go.Figure()
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        x = []
        y = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            x.append(X_Coordinations[node_index])
            y.append(Y_Coordinations[node_index])
            index = solution.Value(routing.NextVar(index))
        node_index = manager.IndexToNode(index)
        x.append(X_Coordinations[node_index])
        y.append(Y_Coordinations[node_index])
        fig.add_trace(go.Scatter(x=x, y=y,
                                 mode='lines+markers',
                                 name=f'route for vehicle {vehicle_id}'))

    fig.show()

def main():
    start = timer()

    X_Coordinations = np.array([82, 96, 50, 49, 13, 29, 58, 84, 14, 2, 3, 5, 98, 84, 61, 1, 88, 91, 19, 93, 50, 98, 5, 42, 61, 9, 80, 57, 23, 20, 85, 98])
    Y_Coordinations = np.array([76, 44, 5, 8, 7, 89, 30, 39, 24, 39, 82, 10, 52, 25, 59, 65, 51, 2, 32, 3, 93, 14, 42, 9, 62, 97, 55, 69, 15, 70, 60, 5])

    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(X_Coordinations, Y_Coordinations)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Setting search strategy
    # AUTOMATIC Lets the solver select the metaheuristic.
    # GREEDY_DESCENT Accepts improving(cost - reducing) local search neighbors until a local minimum is reached.
    # GUIDED_LOCAL_SEARCH Uses guided local search to escape local minima(cf.http: // en.wikipedia.org / wiki / Guided_Local_Search); this is generally the most efficient metaheuristic ->
    # SIMULATED_ANNEALING Uses simulated annealing to escape local minima(cf.http: // en.wikipedia.org / wiki / Simulated_annealing).
    # TABU_SEARCH Uses tabu search to escape local minima(cf.http: // en.wikipedia.org / wiki / Tabu_search).
    # OBJECTIVE_TABU_SEARCH Uses tabu search on the objective value of solution to escape local minima
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
    search_parameters.time_limit.seconds = 30
    search_parameters.log_search = True

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
        plot_solution(data, manager, routing, solution, X_Coordinations, Y_Coordinations)

    end = timer()
    print(end - start) # Time in seconds

if __name__ == '__main__':
    main()