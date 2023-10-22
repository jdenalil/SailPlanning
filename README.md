# SailPlanning
path planning for efficient sailing 

## Problem Statement 
We would like to arrive at some goal location at some time using the minimum power, given the following:
1. Goal relative position and arrival time
1. Magnitude and direction of ocean current
1. Power use vs. water speed profile for the craft, including a maximum speed

### Implementation Overview:
The Solution in this repo uses A* search with modified functions for transition dynamics, heuristic, and success. The goal of the search is to minimize power use. 

#### Success function
Search Nodes are composed of an x position, y position, and a time - making this more similar to a 3-d search problem. The search is not complete until a node matches the goal nodes position and time. 

#### Transition function
There a two cases for generation of possible transition nodes:
1. The boat is at the goal location: In this case, the only transition option available is to hold at the goal location by sailing in opposite direction and speed to the current
2. The boat is not at the goal location: In this case, 8 adjacent locations for the boat are generated. For each of these locations, 10 boat water speeds are used to attempt to sail towards the location. This gives 80 possible (position, time) transition nodes, each with a corresponding power cost. In many cases, it is not possible to sail to a node at a certain boat water speeds given the current. Nodes where the transition is not possible or the time is greater than the goal time are filtered out from the search.

#### Heuristic function
The heuristic used for the search is calculated by adding two components:
1. The power used by the boat sailing at full speed directly towards the goal location
2. The power used by holding at the goal location until the goal time for the remaining time

NOTE: for A* to be optimal, the heuristic must not be greater than the real cost. This is not currently the case for most energy models.

#### Traversal Time function
Both the heuristic and the transition functions leverage the function `calculate_traversal_time` under the hood, which I consider to be the core functionality of this approach. This function takes a current point, a goal point, an ocean current, and a desired boat water speed and calculates the amount of time it will take to traverse between the points using a newton's method solver. This time can be combined with the boat water speed and power map function to calculate the power used over the traversal.

### Why not other planning algorithms?
1. dijkstra: For any non-trivial planning, dijkstra takes a prohibitive amount of time given that there is an equation solver to calculate the cost between two points. If we ignored ocean current, we would have a closed form solution for traversal time and dijkstra would be viable.
1. rrt: TODO

## Future Work
1. Build a low level planner to smooth the discretized waypoints and account for realistic vehicle dynamics
1. Build a quick visualization  of the path taken by the aircraft
1. Allow the number of speeds tried to be given as an input by the user instead of embedded in the code
1. Fix the A* heuristic function to ensure it is always lower then the actual cost
1. Becuase of the equation solver in the traversal time estimator, the algorithm could be quite slow for large inputs. Finding a good approximate closed form solution would be helpful.
1. If it is not possible to get to the goal within the provided time, the search seems to hang. We should make sure the input is valid before searching.
1. Build out unit and integration tests
