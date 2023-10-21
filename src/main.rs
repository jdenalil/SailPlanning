mod astar;

use crate::astar::{Pos, run_astar};

// TODO: what if it is more efficient to go fast and then slow instead of medium, becuase of the power curve dynamics?
// TODO: function to visualize the path taken
// TODO: create a lower level planner that smoothes the waypoints and builds dubins curves

fn main() {
  // we're given a relative goal point, time of arrival, ocean current, and max_speed
  let goal = Pos(90, 120);
  let current_speed = 3.0;
  let current_direction = 0.0; // from north
  let time_goal = 50.0;
  let max_speed = 10.0;

  // run optimizations
  let speed = find_optimal_speed(goal, current_speed, current_direction, time_goal, max_speed);
  println!("found speed {}", speed);
  // calculate watts to go that speed
}


fn find_optimal_speed(goal: Pos, current_speed: f64, current_direction: f64, time_goal: f64, max_speed: f64) -> f64 {
  // find the lowest water speed that will get us to the goal in time
  // first, guess the upper bound to made sure it is possible to reach the target
  if run_astar(goal, current_speed, current_direction, max_speed) == -1.0 {
    println!("search not possible, even at max speed");
    return -1.0
  }

  // okay it's possible, let's find the optimal speed using binary search
  let mut low_speed = 0.0;
  let mut high_speed = max_speed;
  let mut mid_speed = high_speed;
  let mut solved_arrival_time = -1.0;

  while (time_goal - solved_arrival_time).abs() > 0.1 && (high_speed - low_speed) > 0.0001  { 
    mid_speed = (low_speed + high_speed) / 2.0;
    solved_arrival_time = run_astar(goal, current_speed, current_direction, mid_speed);
    // println!("{}, {}", solved_arrival_time, mid_speed);
    
    if solved_arrival_time == -1.0 {
      // search failed bc it is too slow
      low_speed = mid_speed;
    } else if time_goal > solved_arrival_time {
      high_speed = mid_speed;
    } else {
      low_speed = mid_speed;
    }
  }
  mid_speed
}
