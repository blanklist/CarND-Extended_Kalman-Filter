#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;

  // initialize transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
  			 0, 1, 0, 1,
  			 0, 0, 1, 0,
  			 0, 0, 0, 1;
  
  // intialize covaricane matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
  			 0, 1, 0, 0,
  			 0, 0, 1000, 0,
  			 0, 0, 0, 1000;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);

      // polar to cartesian
      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);
      ekf_.x_(2) = rho_dot * cos(phi);
      ekf_.x_(3) = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float elapsed_time = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // update transition matrix
  ekf_.F_(0, 2) = elapsed_time;
  ekf_.F_(1, 3) = elapsed_time;

  // acceleration noise
  float noise_ax = 9;
  float noise_ay = 9;

  // process covariance matrix Q initialization with noise
  float elapsed_time_4 = pow(elapsed_time, 4)
  float elapsed_time_3 = pow(elapsed_time, 3)
  float elapsed_time_2 = pow(elapsed_time, 2)

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << elapsed_time_4 / 4 * noise_ax, 0, elapsed_time_3 / 2 * noise_ax, 0,
  			 0, elapsed_time_4 / 4 * noise_ay, 0, elapsed_time_3 / 2 * noise_ay,
  			 elapsed_time_3 / 2 * noise_ax, 0, elapsed_time_2 * noise_ax, 0,
  			 0, elapsed_time_3 / 2 * noise_ay, 0, elapsed_time_2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
  	// Radar updates
  	ekf_.R_ = R_radar_;
  	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
