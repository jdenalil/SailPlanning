# SailPlanning
path planning for efficient sailing 

## Implementation Details: 

### Problem Statement 
We would like to arrive at some goal location at some time using the minimum power, given the following:
1. Goal relative position and arrival time
1. Magnitude and Direction of Ocean Current
1. Power Use vs. water speed profile for the craft 

### Implementation Overview:
The Solution in this repo uses A* search with modified functions for transition dynamics, hueristic, and success. The goal of the search is to minimize power use. 

#### Success function
Search Nodes are composed of an x position, y position, and a time - making this more similar to a 3-d search problem. The search is not complete until a node matches the goal nodes position and time. 

#### Transition function
There a two cases for generation of possible trasition nodes
1. The boat is at the goal location: In this case, the only transition option available is to hold at the goal location by sailing in opposite direction and speed to the current
2. The boat is not at the goal location: In this case, 8 adjacent locations for the boat are generated. For each of these locations, 10 boat water speeds are used to attempt to sail towards the location. This gives 80 possible (position, time) transition nodes, each with a corresponding power cost. In many cases, it is not possible to sail to a node at a certain boat water speeds given the current. Nodes where the transition is not possible or the time is greater than the goal time are filtered out from the search.

#### Hueristic function
The hueristic use for the search is calculated by adding two components:
1. The power used by the boat sailing at full speed directly towards the goal location
2. The power used by holding at the goal location until the goal time 

NOTE: for A* to be optimal, the hueristic must not be greater than the real cost. This is not currently the case. 

### Why not other planning algorithms?
TODO 



## Future Work
1. The algorithm produces a set of discretized waypoints that don't take into account speed or direction changes - a lower level planner should be added to smooth these waypoints to account for vechicle dynamics
1. Build a quick vizualization of the path taken by the aircraft
1. Allow the energy function to be given as an input by the user instead of embedded in the code
1. Allow the number of speeds tried to be given as an input by the user instead of embedded in the code
1. Fix the hueristic function to ensure it is always lower then the actual cost
