mod astar_variable_speed;
mod dynamics;

use crate::astar_variable_speed::{PosTime, Current, run_astar, SCALING_FACTOR};

// TODO: function to visualize the path taken
// TODO: create a lower level planner that smoothes the waypoints and builds dubins curves
// TODO: fill out README.md

fn main() {
  // we're given a relative goal point, time of arrival, ocean current, and max_speed
  let goal: PosTime = PosTime(15, 20, 10 * SCALING_FACTOR as u32); // goal location is relative to origin
  let current = Current {magnitude: 1.0, direction: 0.7}; // direction measured from north
  let max_speed = 10.0;

  // run optimizations
  let result = run_astar(goal, current, max_speed);
  //let result: Option<(Vec<UnscaledPosTimeSpeed>, f64)>  = None;
  match result {
    Some(t) => {
      for p in t.0 {
        println!("x {} y {} time {}", p.x, p.y, p.time);
      }
      println!("found power cost {}", t.1);
    }
    None => {}
  }
}
