import copy
import random
import sys
import json
import time

instance_path = sys.argv[1]
output_path = sys.argv[2]

with open(instance_path) as instance_file:
    instance = json.load(instance_file)

# requirement: after each iteration of your solver, check that you have not exceeded the time limit (3 minutes) or
# finish the search

# MY SOLUTION CONFIG: greedy initialization: init_sol(); greedy repair: repair(); destroy: shaw_removal(); LNS: lns()

# Store necessary information about current instance
vehicle_capacity = instance['VehicleCapacity']
distance_matrix = instance['DistanceMatrix']
num_of_vehicles = instance['NumberOfVehicles']
location_demands = instance['LocationDemands']


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# INITIALIZATION FUNCTIONS                                            #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def random_init():
    """
    Randomly initialize a solution for the CVRP problem.

    Returns:
        list: A randomly initialized solution represented as a list of routes.
    """
    unvisited = list(range(1, len(location_demands)))
    random.shuffle(unvisited)
    solution = [[0, 0] for _ in range(num_of_vehicles)]

    for customer in unvisited:
        was_placed = False
        while not was_placed:
            # Randomly select a route for a current customer
            route_idx = random.randint(0, len(solution) - 1)

            # Select a position within the route to add the customer
            if len(solution[route_idx]) > 2:
                customer_idx = random.randint(1, len(solution[route_idx]) - 2)
            else:
                customer_idx = 1

            former_route = compute_route_capacity(solution[route_idx])

            # Check if adding the customer does not violate the capacity constraint
            if former_route + location_demands[customer] <= vehicle_capacity:
                # Insert the customer to randomly picked route and index
                (solution[route_idx]).insert(customer_idx, customer)
                was_placed = True

    return solution


def fill_until_possible_init():
    """
    Initialize a solution of CVRP problem by filling routes until it is no longer possible to add more customers,
    while not violating the capacity constraint.

    Returns:
        list: An initialized solution represented by a list of routes

    """
    unvisited = list(range(1, len(location_demands)))
    random.shuffle(unvisited)
    solution = [[0, 0] for _ in range(num_of_vehicles)]

    while unvisited:
        for route in solution:
            for customer in unvisited:

                # Insert the customer to the route if it does not exceed the capacity
                if compute_route_capacity(route) + location_demands[customer] <= vehicle_capacity:
                    route.insert(-1, customer)
                    unvisited.remove(customer)
    return solution


def init_sol():
    """
    Initialize a solution for CVRP problem using greedy approach.

    Return:
        list: An initialized solution represented as a list of routes

    """
    unvisited = list(range(1, len(location_demands)))
    solution = [[0, 0] for _ in range(num_of_vehicles)]

    return greedy_approach(solution, unvisited)


def greedy_approach(solution, customers_to_add):
    """
    Help to initialize the solution by inserting customers into routes using greedy approach.

    Args:
        solution (list): Current version of the solution represented as a list of routes.
        customers_to_add (list): list of unvisited customers to add to the solution.

    Returns:
        list: A solution represented as a list of routes.
    """
    random.shuffle(customers_to_add)
    for customer in customers_to_add:
        best_route = None
        best_position = None
        best_distance = float('inf')

        for route in solution:

            # Calculate the extra cost of inserting the customer between origin and destination
            for i in range(1, len(route)):
                origin = route[i - 1]
                destination = route[i]

                distance_to_customer = distance_matrix[origin][customer]
                distance_from_customer = distance_matrix[customer][destination]
                original_edge_distance = distance_matrix[origin][destination]
                additional_distance = distance_to_customer + distance_from_customer - original_edge_distance

                if (additional_distance < best_distance) and check_constraints(route, customer):
                    best_distance = additional_distance
                    best_route = route
                    best_position = i

        if best_route is not None:
            # Insert the unvisited customer into the selected route at the optimal position
            best_route.insert(best_position, customer)

    return solution


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# REPAIR FUNCTION                                                     #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def repair(solution, removed_customers):
    """
    Repairs solution by inserting the customers back into routes using a greedy approach.
    Args:
         solution (list): Current solution represented as a list of routes, each route containing IDs of customers.
         removed_customers (list): List of removed customers' IDs to be inserted back into the solution

    Returns:
        list: A repaired solution as a list of routes.
    """
    return greedy_approach(solution, removed_customers)


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# DESTROY FUNCTIONS                                                   #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def shaw_removal(solution, num_customers_to_remove, p):
    """
    Apply a Shaw Removal heuristic to remove a specific number of customers from the solution.

    Args:
        solution (list): Current solution represented as a list of routes.
        num_customers_to_remove (int): The number of customers to be removed from the solution.
        p (int): A parameter controlling determinism within the Shaw Removal.

    Returns:
        (list, list): A pair of partially destroyed solution and a customers that were removed from the solution.
    """

    # Creating list of all customers, excluding depot, as it can not be destroyed
    all_customers = list(range(1, len(location_demands)))
    D = []

    # Randomly select an initial customer r to remove and add it to D
    r = random.randint(1, len(location_demands) - 1)
    D.append(r)
    all_customers.remove(r)

    while len(D) < num_customers_to_remove:
        r = random.choice(D)

        L = []
        for customer in all_customers:
            # Create a list of (customer, distance) pairs from r to each remaining customer
            L.append((customer, distance_matrix[r][customer]))

        # Sort L by increasing distance
        L.sort(key=lambda x: x[1])

        # Choose a random customer from L using y and p and add it to D
        y = random.random()
        idx = int((y ** p) * (len(L)))
        D.append((L[idx])[0])
        all_customers.remove((L[idx])[0])

    # Remove the customers in D from solution
    for customer_to_remove in D:
        for route in solution:
            if customer_to_remove in route:
                route.remove(customer_to_remove)
                break

    return solution, D


# it can in case that all the new solutions are worst return worse solution
def worst_removal(solution, num_customers_to_remove):
    """
    Apply the Worst Removal heuristic to remove a specified number of customers from the solution.

    Args:
        solution (list): A current solution represented as a list of routes.
        num_customers_to_remove (int): A number of customers to be removed from the solution.

    Returns:
        (list, list): A pair of solution with removed customers and a list of customers' IDs that were removed.
    """
    removed_customers = []

    for _ in range(num_customers_to_remove):
        max_cost = -float('inf')
        customer_to_remove = None

        for route in solution:
            original_cost = total_distances(route)

            for customer in route[1:-1]:
                route_copy = route[:]
                route_copy.remove(customer)

                after_removal_cost = total_distances(route_copy)

                cost = original_cost - after_removal_cost
                if cost >= max_cost:
                    max_cost = cost
                    customer_to_remove = customer

        # Remove the customer with the worst performance
        for route in solution:
            if customer_to_remove in route:
                route.remove(customer_to_remove)
                removed_customers.append(customer_to_remove)

    return solution, removed_customers


def random_removal(solution, num_to_remove):
    """
    Apply a Random Removal heuristic to randomly remove a specified number of customers from the solution.

    Args:
        solution (list): A current solution represented as a list of routes.
        num_to_remove (int): A number of customers to be removed from the solution.

    Returns:
        (list, list): A tuple of partially destroyed solution and customers that were removed from the solution.

    """
    s = copy.deepcopy(solution)
    customer = []

    for _ in range(num_to_remove):

        # Randomly select a customer from a non-empty route
        route_idx = random.randint(0, len(s) - 1)
        while len(s[route_idx]) <= 2:
            route_idx = random.randint(0, len(s) - 1)

        # Randomly select a customer within the route to remove (excluding depot)
        customer_idx = random.randint(1, len(s[route_idx]) - 2)

        # Remove the selected customer from the route
        customer.append(s[route_idx].pop(customer_idx))

    return s, customer


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# OBJECTIVE FUNCTION                                                  #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def obj_f(s):  # objective function based on the total distance
    """
    Calculate the objective function value based on the total travel distance of a given solution.

    Args:
        s (list): Current solution represented as a list of routes.

    Returns:
        int: The total travel distance.
    """
    total_distance = 0

    # Calculate the total travel distance of the solution.
    for route in s:

        # Starting at depot
        curr_location = 0

        for elem in route:
            total_distance += distance_matrix[curr_location][elem]  # calculate_route_distance(route, instance)
            curr_location = elem

    return total_distance


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# OTHER FUNCTIONALITY                                                 #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def check_if_all_nums_once(s_prime):
    """
    Check if all customers' IDs appear exactly once in solution routes.

    Args:
        s_prime (list): Current solution represented as a list of routes.

    Returns:
        bool: True if all customers' IDs appear exactly once in solution routes, else False.
    """
    all_customers = list(range(1, len(location_demands)))
    flatten = []
    for route in s_prime:
        for customer in route:
            if customer != 0:
                flatten.append(customer)

    if len(all_customers) != len(flatten):
        return False

    flatten.sort()
    for i in range(len(all_customers)):
        if all_customers[i] != flatten[i]:
            return False

    return True


def check_for_capacity(s_prime):
    """
    Check if the capacity constraint is violated in the given solution.

    Args:
        s_prime (list): A current solution represented by a list of routes.

    Returns:
        bool: True if the capacity is violated, else False.
    """
    capacity = vehicle_capacity
    is_exceeded = False
    for route in s_prime:
        curr_route_capacity = 0
        for elem in route:
            curr_route_capacity += location_demands[elem]

        if curr_route_capacity > capacity:
            print("LIMIT EXCEEDED FOR: " + str(route) + " " + str(curr_route_capacity))
            is_exceeded = True
            break

    return is_exceeded


def check_zeros(s):
    """
    Check if each route in the solution starts and ends with a 0 and has exactly 2 zeros.

    Args:
        s (list): A current solution represented by a list of routes.
    """
    y = True
    for route in s:
        count_zeros = 0
        for cust in route:
            if cust == 0:
                count_zeros += 1
        if count_zeros != 2:
            y = False
            break
    z = True
    for route in s:
        if route[0] != 0 or route[-1] != 0:
            z = False

    print("ROUTE STARTS AND ENDS WITH 0: ", y)
    print("THERE ARE EXACTLY TWO 0s IN EACH ROUTE: ", z)


def check_num_of_vehicles(s):
    """
    Check if the number of routes in the solution corresponds to number of vehicles

    Args:
        s (list): A current solution represented by list of routes.
    """
    print("CORRECT NUMBER OF VEHICLES: ", len(s) == num_of_vehicles)


def compute_route_capacity(route):
    """
    Calculate a total capacity demands within given route

    Args:
        route (list): Route that consists of customers' IDs that are part of that particular route.

    Returns:
        int: Capacity demands within given route.
    """
    route_capacity = 0
    for elem in route[1:-1]:
        route_capacity += location_demands[elem]
    return route_capacity


def total_distances(route):
    """
    Calculate a total traveled distance in a given route.

    Args:
        route (list): Route that consists of customers' IDs that are part of that particular route.

    Returns:
        int: The total traveled distance within particular route.
    """
    dist = 0
    for i in range(len(route) - 1):
        dist += distance_matrix[route[i]][route[i + 1]]
    return dist


def check_constraints(route, customer):
    """
    Check if adding a customer to a route would not violate vehicle capacity constraint.

    Args:
        route (list): Route that consists of customers' IDs that are part of that particular route.
        customer (int): ID of the customer to be added to a route.

    Returns:
        bool: True if adding the customer to a given route does not violate vehicle capacity, False otherwise.
    """
    total_demand = 0
    for elem in route:
        total_demand += (location_demands[elem])
    return (total_demand + location_demands[customer]) <= vehicle_capacity


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# OPTIMIZATION METHODS;                                               #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def two_opt(route):
    """
    Apply the 2-opt heuristic to improve a given route by swapping pairs of edges.

    Args:
        route (list): The current route represented as a list of customer IDs.

    Returns:
        list: The improved route after applying the 2-opt heuristic.
    """
    best_route = route
    improved = True

    while improved:
        improved = False

        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1:
                    continue  # no reversal would be possible
                new_route = route[:]
                new_route[i:j] = route[j - 1:i - 1:-1]  # reverse the subsection between i and j

                if total_distances(new_route) < total_distances(best_route):
                    best_route = new_route
                    improved = True

        route = best_route

    return best_route


def two_opt_b(route):
    """
    Apply the 2-opt heuristic to improve a given route by swapping pairs of edges.

    Args:
        route (list): The current route represented as a list of customer IDs.

    Returns:
        list: The improved route after applying the 2-opt heuristic.
    """
    best_route = route
    best_distance = total_distances(route)
    improved = True
    num_iterations_without_improvement = 0

    while improved:
        improved = False

        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):

                if j - i == 1:
                    continue  # no reversal would be possible
                new_route = route[:]
                new_route[i:j] = route[j - 1:i - 1:-1]  # reverse the subsection between i and j
                new_distance = best_distance - distance_matrix[route[i - 1]][route[i]] - distance_matrix[route[j - 1]][
                    route[j]] + distance_matrix[route[i - 1]][route[j - 1]] + distance_matrix[route[i]][route[j]]

                if new_distance < best_distance:
                    best_route = new_route
                    best_distance = new_distance
                    improved = True
                    num_iterations_without_improvement = 0
        route = best_route
        num_iterations_without_improvement += 1

        # Terminate the loop if no improvement has been found after a certain number of iterations
        if num_iterations_without_improvement >= 20:
            break

    return best_route


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# LARGE NEIGHBORHOOD SEARCH                                           #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def lns(a=250, p=5):  # 5; 250
    """
    Perform Large Neighborhood Search (LNS) in order to find improved solution.

    Args:
        a (int): If solution is not improved during this number of iterations, the degree of destruction is increased.
        p (int): A parameter controlling determinism within the Shaw Removal.

    Returns:
        list: The improved solution after LNS.
    """

    # Initialize solution
    s = init_sol()

    s_best = s
    cost_s_best = obj_f(s_best)  # compute cost of best solution

    print("COST OF INITIAL SOLUTION: ", cost_s_best)
    print("-------------------------------------")

    num_nb_to_remove = 1
    max_nb_to_remove = round(0.2 * (len(location_demands) - 1))  # 0.15
    does_cost_improve = False
    does_best_sol_change = False

    total_time = 0
    i = 0
    num_iterations_not_improved = 0

    while total_time < 170:
        i += 1
        start_time = time.time()

        s_prime = copy.deepcopy(s)
        s_prime = shaw_removal(s_prime, num_nb_to_remove, p)
        s_prime = repair(s_prime[0], s_prime[1])

        # Route optimization step
        for route_idx, route in enumerate(s_prime):
            if len(route) > 2:  # Check if the route is not empty
                s_prime[route_idx] = two_opt_b(route)

        # Compute cost of s_prime
        cost_s_prime = obj_f(s_prime)

        # Compute cost of s_best only if it has changed from previous iteration
        if does_best_sol_change:
            cost_s_best = obj_f(s_best)

        # Replace s_best for s_prime if cost of s_prime is smaller than of the current s_best
        if cost_s_prime < cost_s_best:
            s_best = s_prime
            num_iterations_not_improved = 0
            does_best_sol_change = True
        else:
            num_iterations_not_improved += 1
            does_best_sol_change = False

        # Compute cost of s
        cost_s = obj_f(s)

        # Accept only improving solution
        if cost_s_prime < cost_s:
            does_cost_improve = True
            s = s_prime

        # If the solution did not improve in number of iterations given by a, increment degree of removal
        if i % a == 0:
            if not does_cost_improve:
                if num_nb_to_remove <= max_nb_to_remove:
                    num_nb_to_remove += 1
            does_cost_improve = False
        end_time = time.time()
        total_time += (end_time - start_time)

        if num_iterations_not_improved >= 4000:
            print("BREAK")
            break

    return s_best


def lns_b(a=250, number_of_iterations=10000, reduction_factor=0.97, threshold=0.01):
    """
    Preform Large Neighborhood Search (LNS) in order to find improved solution.

    Args:
        a (int): If solution is not improved during this number of iterations, the degree of destruction is increased.
        number_of_iterations (int): Maximal number of iterations preformed in LNS.
        reduction_factor (float): Reduce the threshold by this factor in each iteration.
        threshold (float): Value that is being reduces by the reduction factor.

    Returns:
        list: The improved solution after LNS.
    """

    s = init_sol()

    s_best = s
    num_nb_to_remove = 1
    max_nb_to_remove = round(0.15 * (len(location_demands) - 1))
    does_cost_improve = False
    difference = 20

    total_time = 0

    for i in range(1, number_of_iterations):  # fixed number of loops

        time_start = time.time()
        if total_time > 170:
            break
        if difference >= 1:
            threshold *= reduction_factor

        # curr_degree_of_destruction = random.randint(min_degree_of_destruction, max_degree_of_destruction)
        s_prime = copy.deepcopy(s)
        s_prime = shaw_removal(s_prime, num_nb_to_remove, 15)
        s_prime = repair(s_prime[0], s_prime[1])

        # Apply route optimization step
        for route_idx, route in enumerate(s_prime):
            if len(route) > 2:  # Check if the route is not empty
                s_prime[route_idx] = two_opt(route)

        cost_s_prime = obj_f(s_prime)
        if cost_s_prime < obj_f(s_best):  # accepting only solutions that "get better"
            s_best = s_prime

        cost_s = obj_f(s)
        difference = round(cost_s * threshold)

        # Accept improving solution
        if cost_s_prime < cost_s:
            does_cost_improve = True
            s = s_prime

        # Accept also non-improving solution in case the cost_s_prime - cost_s do not exceed the given threshold
        elif cost_s_prime - cost_s < difference:
            s = s_prime

        if i % a == 0:
            if not does_cost_improve:
                if num_nb_to_remove <= max_nb_to_remove:
                    num_nb_to_remove += 1
            does_cost_improve = False
        time_stop = time.time()
        total_time += (time_stop - time_start)

    return s_best


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# main                                                                #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
def main():
    best_solution = lns()
    print("ARE ALL NUMS PRESENT EXACTLY ONCE: " + str(check_if_all_nums_once(best_solution)))
    print("DID SOME VEHICLE EXCEED CAPACITY? " + str(check_for_capacity(best_solution)))
    check_num_of_vehicles(best_solution)
    check_zeros(best_solution)
    print("----------------------------------------------------")
    print("COST OF OPTIMIZED SOLUTION: ", obj_f(best_solution))

    with open(output_path, 'w') as output_file:
        json.dump(best_solution, output_file)


if __name__ == "__main__":
    main()
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# FINE-TUNING                                                         #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Grid search; searching for optimal values p and a
"""
p_pool = [1, 5, 10, 15, 30]
a_pool = [250, 500, 1000]
instances_paths = \
    [
     "path/to/instances"
     ]

output_path = "path/to/output"
for inst in instances_paths:
    with open(inst) as f:
        instance = json.load(f)
        vehicle_capacity = instance['VehicleCapacity']
        distance_matrix = instance['DistanceMatrix']
        num_of_vehicles = instance['NumberOfVehicles']
        location_demands = instance['LocationDemands']
        for curr_p in p_pool:
            for curr_a in a_pool:
                curr_sol = lns(curr_a, curr_p)
                curr_cost = obj_f(curr_sol)
                write_down = ("INSTANCE cvrp:", len(location_demands), ", PARAMETER P:", curr_p,
                      "PARAMETER A", curr_a, ", TOTAL COST", curr_cost)
                print(write_down)
                with open(output_path, 'a') as f_2:
                    json.dump(write_down, f_2)

"""

'''
def hill_climbing(initial_parameters, max_iterations):
    """
    Helps to finetune the parameters for LNS

    Args:
        initial_parameters (lst): List of initial parameters.
        max_iterations (int): Maximal number of iterations when preforming hill climbing.

    Returns:
        list: Parameters for which the cost function was smallest
    """
    current_parameters = initial_parameters
    initial_sol = lns_b(a=current_parameters[0], number_of_iterations=current_parameters[1],
                     reduction_factor=current_parameters[2], threshold=current_parameters[3])
    current_score = obj_f(initial_sol)

    for i in range(max_iterations):
        print(i)
        neighbor_parameters = generate_neighbor(current_parameters)
        neighbor_sol = lns_b(a=neighbor_parameters[0], number_of_iterations=neighbor_parameters[1],
                         reduction_factor=neighbor_parameters[2], threshold=neighbor_parameters[3])
        neighbor_score = obj_f(neighbor_sol)

        if neighbor_score < current_score:  # Minimize the total cost
            current_parameters = neighbor_parameters
            current_score = neighbor_score

    return current_parameters


def generate_neighbor(current_parameters):
    """
    Perturbs randomly selected parameters.

    Args:
        current_parameters (lst): List of parameter from which to choose one to be perturbed.

    Returns:
        list: List of parameters with one perturbed parameter.
    """

    # Replace this with logic to generate neighboring parameters
    neighbor = current_parameters.copy()
    index = random.randint(0, len(neighbor) - 2)

    # Perturb a random parameter
    if index == 0:
        neighbor[index] += (max(1, round(random.uniform(-50, 50))))

    elif index == 1:
        neighbor[index] += round(random.uniform(-100, 100))

    elif index == 2:
        neighbor[index] += round(random.random() * (0.999 - 0.9) + 0.9, 3)

    elif index == 3:
        neighbor[index] = 0.01  # given in paper from Shaw

    return neighbor
'''
