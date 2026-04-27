# Instance format

The provided instances are saved as JSON files with the following attributes:

 * __Coordinates__: list of all location coordinates (two-element lists [x, y]), indexed by location ID
 * __LocationDemands__: list of location demands, i.e., the amount of load to be transported from depot to the location, indexed by location ID
 * __DistanceMatrix__: Euclidean 2D matrix of distances between all pairs of locations (symmetric, triangle inequality), indexed by location ID
 * __VehicleCapacity__: integer limit on the amount of load that can be transported in one vehicle
 * __NumberOfVehicles__: integer defining the fleet size
 * __GlobalBestSolution__: reference best known solution
 * __GlobalBestTotalDistance__: reference value of global optimum or best known solution


The location ID of depot is always 0, customer location IDs start from 1. The first (index 0) element of __CustomerDemands__ is always 0 (for the depot) and only serves as a padding so that all arrays may be indexed in the same manner, i.e., by location ID.

The point of providing you with __GlobalBestSolution__ and __GlobalBestTotalDistance__ is to allow for comparison of the found solution with an ideal one. It is forbidden to use any of these fields in the code of your solver.

Examples of routes:

[0, 0]			empty route
[0, 1, 2, 3, 4, 0] 	route visiting customers 1 to 4

# Solution feasibility and objective

A feasible solution must satisfy the following conditions:

 * Each customer (locations other than depot) is visited exactly once
 * Sum of demands across all visited customers is less than or equal to __VehicleCapacity__ in all routes
 * Each route visits the depot exactly at its start and end
 
 The quality of a solution is measured as the total distance traveled by the whole vehicle fleet based on the distance information provided in __DistanceMatrix__.

