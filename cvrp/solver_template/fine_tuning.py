import random
import json

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# FINE-TUNING                                                         #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Grid search; searching for optimal values p and a

p_pool = [1, 5, 10, 15, 30]
a_pool = [250, 500, 1000]
instances_paths = \
    [
        "path/to/instance.json"
    ]

output_path = "output/path"
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
        neighbor_solution = lns_b(a=neighbor_parameters[0], number_of_iterations=neighbor_parameters[1],
                                  reduction_factor=neighbor_parameters[2], threshold=neighbor_parameters[3])
        neighbor_score = obj_f(neighbor_solution)

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
