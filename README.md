# google-or-tools
Experimenting with Google OR-tools

https://developers.google.com/optimization/routing

A more general version of the TSP is the vehicle routing problem (VRP), in which there are multiple vehicles. In most cases, VRPs have constraints: for example, vehicles might have capacities for the maximum weight or volume of items they can carry, or drivers might be required to visit locations during specified time windows requested by customers. OR-Tools can solve many types of VRPs, including the following:

Traveling salesman problem, the classic routing problem in which there is just one vehicle.
Vehicle routing problem, a generalisation of the TSP with multiple vehicles.
VRP with capacity constraints, in which vehicles have maximum capacities for the items they can carry.
VRP with time windows, where the vehicles must visit the locations in specified time intervals.
VRP with resource constraints, such as space or personnel to load and unload vehicles at the depot (the starting point for the routes).
VRP with dropped visits, where the vehicles aren't required to visit all locations, but must pay a penalty for each visit that is dropped.

Note: for large TSP problems this could be a better solution: http://www.math.uwaterloo.ca/tsp/concorde.html


## issues

- capacity constraints can be used to deal with different kind of loads e.g. passengers and wheelchairs
- pickup and delivery can be used to deal with on-demand mode
- resource constraints could be used to deal with battery capacity
