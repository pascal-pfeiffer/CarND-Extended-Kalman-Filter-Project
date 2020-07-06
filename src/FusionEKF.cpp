#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,  // x_coord
              0, 0.0225;  // y_coord

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,  // rho
              0, 0.0009, 0,   //phi
              0, 0, 0.09;  // rho_dot

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // initial state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,  // x_coord
             0, 1, 0, 0,  // y_coord
             0, 0, 1000, 0,  // vx
             0, 0, 0, 1000;  // vy

  // initial state covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;
  
  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,  // x_coord
              0, 1, 0, 0;  // y_coord

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;  // x_coord, y_coord, vx , vy

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      cout << "RADAR is first measurement" << endl;
      // To convert from Polar Coordinates (rho, phi) to Cartesian Coordinates (x_coord, y_coord) :
      //   x = rho × cos( phi )
      //   y = rho × sin( phi )
      float rho = measurement_pack.raw_measurements_[0];  // range
      float phi = measurement_pack.raw_measurements_[1];  // counter-clockwise from x-axis
      float rho_dot = measurement_pack.raw_measurements_[2];  // The range rate, is the projection of the velocity onto the line
      
      float cos_phi = cos(phi);
      float sin_phi = sin(phi);
      
      float x_coord = rho * cos_phi;
      float y_coord = rho * sin_phi;
      
      float vx = rho_dot * cos_phi;
      float vy = rho_dot * sin_phi;
      
      // cout << x_coord << y_coord << vx << vy << endl;
      
      // and initialize state.
      ekf_.x_ << x_coord, y_coord, vx, vy;
      
      // Modify the P matrix that the covariance in speed is better
      ekf_.P_(2, 2) = 1;  // vx
      ekf_.P_(3, 3) = 1;  // vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "LIDAR is first measurement" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0],  // x
                 measurement_pack.raw_measurements_[1],  // y
                 0,  // vx  (Lidar has no velocity information)
                 0;  // vy  (Lidar has no velocity information)
    }
    
    // initialise time
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // cout << "dt: " << dt << endl;
  
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update the process covariance matrix Q
  float noise_ax = 9;  // as stated in the comments above
  float noise_ay = 9;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  float dt_44 = dt_4 / 4;
  float dt_32 = dt_3 / 2;
  
  float dt_32_nx = dt_32 * noise_ax;
  float dt_32_ny = dt_32 * noise_ay;
  
  ekf_.Q_ << dt_44*noise_ax, 0, dt_32_nx, 0,
             0, dt_44*noise_ay, 0, dt_32_ny,
             dt_32_nx, 0, dt_2*noise_ax, 0,
             0, dt_32_ny, 0, dt_2*noise_ay;
  
  ekf_.Predict();
  // cout << "Q_ = " << ekf_.Q_ << endl;
  // cout << "F_ = " << ekf_.F_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
  
  // cout << "x_ = " << ekf_.x_ << endl;
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);  // use jacobian H matrix
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    // use laser measurement matrice H 
    // use laser measurement covariance matrix R
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // taken from the course
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
