mod astar_variable_speed;
mod dynamics;

use crate::astar_variable_speed::{PosTimeSpeed, run_astar, UnscaledPosTimeSpeed, calc_power, SCALING_FACTOR};

// TODO: function to visualize the path taken
// TODO: create a lower level planner that smoothes the waypoints and builds dubins curves
// TODO: fill out README.md

fn main() {
  // we're given a relative goal point, time of arrival, ocean current, and max_speed
  let goal: PosTimeSpeed = PosTimeSpeed(15, 20, 10 * SCALING_FACTOR as u32, 0);
  let current_speed = 1.0;
  let current_direction = 0.7; // from north
  let max_speed = 10.0;

  // run optimizations
  let result = run_astar(goal, current_speed, current_direction, max_speed);
  //let result: Option<(Vec<UnscaledPosTimeSpeed>, f64)>  = None;
  match result {
    Some(t) => {
      for p in t.0 {
        println!("x {} y {} speed {} time {}", p.x, p.y, p.speed, p.time);
      }
      println!("found power cost {}", t.1);
    }
    None => {}
  }
}
