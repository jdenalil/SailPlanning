# SailPlanning
path planning for efficient sailing 

## Problem Statement 
We would like to arrive at some goal location at some time using the minimum power, given the following:
1. Goal relative position and arrival time
1. Magnitude and direction of ocean current
1. Power use vs. water speed profile for the craft, including a maximum speed

## Quickstart
To run an example planning scenario, just change the values in `main.rs` and run `cargo run`. This will print the results of the planning. To use the planning results, the main entrypoint is the `run_astar` function, which returns a vector of Postion & Time waypoints and a total power use for the path (`Option<(Vec<PosTime>, u32)>`).

### Disclaimer
This implementation is a work in progress, and does not properly reject invalid inputs or guarantee an optimal solution.

## Implementation Overview:
The Solution in this repo uses A* search with modified functions for transition dynamics, heuristic, and success. The goal of the search is to minimize power use. 

### Success function
Search Nodes are composed of an x position, y position, and a time - making this more similar to a 3-d search problem. The search is not complete until a node matches the goal nodes position and time. 

### Transition function
There a two cases for generation of possible transition nodes:
1. The boat is at the goal location: In this case, the only transition option available is to hold at the goal location by sailing in opposite direction and speed to the current
2. The boat is not at the goal location: In this case, 8 adjacent locations for the boat are generated. For each of these locations, 10 boat water speeds are used to attempt to sail towards the location. This gives 80 possible (position, time) transition nodes, each with a corresponding power cost. In many cases, it is not possible to sail to a node at a certain boat water speeds given the current. Nodes where the transition is not possible or the time is greater than the goal time are filtered out from the search.

### Heuristic function
The heuristic used for the search is calculated by adding two components:
1. The power used by the boat sailing at full speed directly towards the goal location
2. The power used by holding at the goal location until the goal time for the remaining time

NOTE: for A* to be optimal, the heuristic must not be greater than the real cost. This is not currently the case for most energy models.
To mitigate this, I have an option to scale down the traversal power component of the hueristic by some constant value (the power used to hold at location is already a perfect estimate). This is a hand-tuned component, and making the value too high will result in the search runtime exploding, especially when the current is strong.

### Traversal Time function
Both the heuristic and the transition functions leverage the function `calculate_traversal_time` under the hood, which I consider to be the core functionality of this approach. This function takes a current point, a goal point, an ocean current, and a desired boat water speed and calculates the amount of time it will take to traverse between the points using a newton's method solver. This time can be combined with the boat water speed and power map function to calculate the power used over the traversal.

## Current Tech Debt
1. Allow the number of speeds tried to be given as an input by the user instead of embedded in the code.
1. Fix the A* heuristic calculation to ensure it is always lower then the actual cost.
1. Improve rejection of generally invalid inputs like negative maximum boat speeds. For example, if it is not possible to get to the goal within the provided time, the search will hang instead of returning an error message.
1. Add integration tests.

## Future Features
1. Build a low level planner to smooth the discretized waypoints and account for realistic vehicle dynamics.
1. Build a visualization of the path taken by the craft.
1. Accept Current as map of position -> current instead of a constant magnitude and direction.
1. Take waves into account during traversal time calculation. This will entail adding a wave height map over time object that the planner can query with position and time values. Since the waves are changing with time, this will add additional non-linearity to the traversal time.
1. Take into account no-go zones. If these zones can be queried with a position and time, search nodes can easily by filtered out by the planner. 

In summary, having access to a world map with current, wave height, and no-go zones is the logical next step for the planner.