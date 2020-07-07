# Extended Kalman Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

In the following I will briefly discuss the rubrics for this project. 

## Compiling
1. Your code should compile.
   * The Code compiles without errors with `cmake` and `make`.
   * No changes to the `CMakeList.txt` were necessary. 

## Accuracy
1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: `obj_pose-laser-radar-synthetic-input.txt` which is the same data file the simulator uses for Dataset 1.
   * In Dataset 1, px, py, vx, vy output coordinates have an RMSE = [.0974, .0855, 0.4517, 0.4404]
   * In Dataset 2, px, py, vx, vy output coordinates have an RMSE <= [.0740, .0964, 0.4466, 0.4756]

## Follows the Correct Algorithm
1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
   * The algorithm in the project follows the preceding lesson.
2. Your Kalman Filter algorithm handles the first measurements appropriately.
   * The algorithm uses the first measurements to initialize the state vectors and covariance matrices. These may be either lidar or radar measurements. 
3. Your Kalman Filter algorithm first predicts then updates.
   * Upon receiving a measurement after the first, the algorithm predicts object position to the current timestep and then updates the prediction using the new measurement.
4. Your Kalman Filter can handle radar and lidar measurements.
   * The Extended Kalman Filter can handle radar and lidar measurements. 

## Code Efficiency
1. Your algorithm should avoid unnecessary calculations.
   * The code pre computes expressions that are used more than once.
   * Unnecessary for loops are avoided.