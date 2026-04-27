# CVRP Solver using Large Neighborhood Search

This project implements a heuristic solver for the Capacitated Vehicle Routing Problem (CVRP). The solver is based on Large Neighborhood Search (LNS), using greedy insertion for initialization and repair, Shaw removal as the destroy operator, and 2-opt local search for route improvement.

## Algorithm

The default solver pipeline is:

1. Generate an initial solution using greedy insertion.
2. Iteratively destroy part of the solution using Shaw removal.
3. Repair the solution using greedy insertion.
4. Improve each route using 2-opt local search.
5. Keep the best feasible solution found within the time limit.

## Repository structure

```text
data/
    Example input instances.

solver_template/
    Python implementation of the CVRP solver.

README.md
    Project documentation.

```
## Usage
python solver_template/main.py data/example_instance.json output_solution.json


## Input format

Input instances are JSON files. The full instance format is described in [`cvrp/data/README.md`](cvrp/data/README.md).

## Solution format

The solver produce a single JSON file containing one solution. The solution is a list of ``NumberOfVehicles`` routes (including empty routes). Each route is represented as an ordered list of location IDs. The lists start and end with a visit to the depot (location ID 0).

## Requirements

The solver uses only the Python standard library. No additional packages are required.

## Additional experimental components

Besides the default solver pipeline, the code also contains alternative components that were tested during development:

- `random_init()` — random feasible initialization
- `fill_until_possible_init()` — route initialization by filling vehicles while capacity allows
- `random_removal()` — random customer removal destroy operator
- `worst_removal()` — removes the customer whose removal gives the largest route-distance saving
- `lns_b()` — alternative LNS variant with threshold-based acceptance of slightly worse solutions
- commented fine-tuning code for testing parameter combinations

These components are kept for reference and experimentation. The default execution uses greedy initialization, Shaw removal, greedy repair, 2-opt route improvement, and `lns()`.


