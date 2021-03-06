#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
//#include <boost/thread.hpp>
//#include <boost/bind.hpp>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);  
  for(int i=0; i<n_x_;i++) {
      P_(i,i) = 0.105;
  }

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0);

  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.885;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.28;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  is_initialized_ = false;

  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) ||
        (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true)) {

        if (!is_initialized_) {

            //initialize state
            if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) {
                x_(0) = meas_package.raw_measurements_(0);   //px
                x_(1) = meas_package.raw_measurements_(1);   //py
                x_(2) = 0;  //v
                x_(3) = atan2(x_(1), x_(0)); // theta
                x_(4) = 0; // theta_dot
            }
            else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true) {
              float ro = meas_package.raw_measurements_(0);
              float theta = meas_package.raw_measurements_(1);

              x_(0) = ro * cos(theta);  //px
              x_(1) = ro * sin(theta);  //py
              x_(2) = 0;  //v
              x_(3) = atan2(x_(1), x_(0));; //theta
              x_(4) = 0; // theta_dot
            }

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;
    }

    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    //Predict sigma
    Prediction(delta_t);

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && (use_radar_ == true)) {

        UpdateRadar(meas_package);

        // output the NIS values
        //outputNIS(NIS_radar_); 
        //simply output file will damage the effiency of UKF calculation result.
        //boost_thread = new boost::thread(boost::bind(&UKF::outputNIS, this, NIS_radar_));

    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER && (use_laser_ == true)) {

        UpdateLidar(meas_package);

        // output the NIS values
        //outputNIS(NIS_laser_);
        //boost_thread = new boost::thread(boost::bind(&UKF::outputNIS, this, NIS_laser_));

    }

  }
  return;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

    //generate sigma points, this step can be ignored
    //because we have augmented sigma here.
    //MatrixXd Xsig_out = MatrixXd(n_x_, 2*n_x_+1);
    //GenerateSigmaPoints(&Xsig_out);

    //augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
    AugmentedSigmaPoints(&Xsig_aug);

    SigmaPointPrediction(Xsig_aug, delta_t);

    //Predict Mean And Covariance
    PredictMeanAndCovariance( );

    return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    //Predict Measurement
    int n_z = 2;
    VectorXd z_pred = VectorXd(n_z);;
    MatrixXd S_out = MatrixXd(n_z, n_z);;
    MatrixXd Zsig_out = MatrixXd(n_z, 2*n_aug_+1);

    //Predict Measurement
    PredictLaserMeasurement(&z_pred, &S_out, &Zsig_out);

    //Update State
    VectorXd z_meas = VectorXd(n_z);
    z_meas(0) = meas_package.raw_measurements_(0);  //px
    z_meas(1) = meas_package.raw_measurements_(1);  //py

    UpdateLaserState(z_meas, Zsig_out, z_pred, S_out);

    return;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    int n_z = 3;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S_out = MatrixXd(n_z, n_z);
    MatrixXd Zsig_out = MatrixXd(n_z, 2*n_aug_+1);

    //Predict Measurement
    PredictRadarMeasurement(&z_pred, &S_out, &Zsig_out);

    //Update State
    VectorXd z_meas = VectorXd(n_z);
    z_meas(0) = meas_package.raw_measurements_(0);  //ro
    z_meas(1) = meas_package.raw_measurements_(1);  //theta
    z_meas(2) = meas_package.raw_measurements_(2);  //ro_dot

    UpdateRadarState(z_meas, Zsig_out, z_pred, S_out);

    return;
}


void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
    MatrixXd Xsig = MatrixXd(n_x_, 2*n_x_+1);
    Xsig.col(0) = x_;
    MatrixXd A = P_.llt().matrixL();

    lambda_ = 3 - n_x_;

    for(int i = 0; i < n_x_; i++) {
        Xsig.col(i+1) = x_ + sqrt(lambda_+n_x_) * A.col(i);
        Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
    }

    //Normalizing angle theta
    /*for(int i = 0; i < 2*n_x_+1; i++) {
        while(Xsig(3,i) < -M_PI) Xsig(3,i) += 2*M_PI;
        while(Xsig(3,i) > M_PI)  Xsig(3,i) -= 2*M_PI;
    }*/
    *Xsig_out = Xsig;
    return;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(5) = x_;
    x_aug(5) = 0; //std_a_ * std_a_;
    x_aug(6) = 0; //std_yawdd_ * std_yawdd_;

    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);

    MatrixXd Q = MatrixXd(2,2);
    Q(0,0) = std_a_ * std_a_;
    Q(1,1) = std_yawdd_ * std_yawdd_;

    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    //P_aug.bottomRightCorner(2,2) = Q(2,2);   //this is wrong, make big overflow. caution!
    P_aug(5, 5) = Q(0, 0);
    P_aug(6, 6) = Q(1, 1);

    MatrixXd A = P_aug.llt().matrixL();

    lambda_ = 3 - n_aug_;

    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
    Xsig_aug.col(0) = x_aug;

    for(int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
    }

    //Normalizing angle theta
    /*for(int i = 0; i < 2*n_aug_+1; i++) {
        while(Xsig_aug(3,i) < -M_PI) Xsig_aug(3,i) += 2*M_PI;
        while(Xsig_aug(3,i) > M_PI)  Xsig_aug(3,i) -= 2*M_PI;
    }*/
    *Xsig_out = Xsig_aug;
    return;
}

void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, double delta_t){

    for(int i = 0; i < 2*n_aug_+1; i++){

        double p_x      = Xsig_aug(0, i);
        double p_y      = Xsig_aug(1, i);
        double v        = Xsig_aug(2, i);
        double yaw      = Xsig_aug(3, i);
        double yawd     = Xsig_aug(4, i);
        double nu_a     = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.0001) {
          px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
          py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else {
          px_p = p_x + v * delta_t * cos(yaw);
          py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }

    return;
}

void UKF::PredictMeanAndCovariance() {
    lambda_ = 3 - n_aug_ ;

    weights_(0) = lambda_/(lambda_ + n_aug_);

    for(int i = 1; i < 2*n_aug_+1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    VectorXd x = VectorXd(n_x_);
    x.fill(0.0);

    for(int i = 0; i < 2*n_aug_+1; i++) {
        x = x + weights_(i) * Xsig_pred_.col(i);
    }

    x_ = x;

    MatrixXd P = MatrixXd(n_x_, n_x_);
    P.fill(0.0);

    for(int i = 0; i < 2*n_aug_+1; i++) {
         VectorXd x_diff = Xsig_pred_.col(i) - x_;

         /*cout << "i = " << i << endl;
         cout << "Xsig_pred_ = " << Xsig_pred_ << endl;
         cout << "x_ = " << x_ << endl;
         cout << "x_diff = " << x_diff << endl;*/

         //Normalizing angle theta
         while(x_diff(3) < -M_PI) x_diff(3) += 2*M_PI;
         while(x_diff(3) > M_PI)  x_diff(3) -= 2*M_PI;

         P = P + weights_(i) * x_diff * x_diff.transpose();
    }

    P_ = P;

    return;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {
    lambda_ = 3 - n_aug_;
    int n_z = 3;

    weights_(0) = lambda_/(lambda_ + n_aug_);
    for(int i = 1; i < 2*n_aug_+1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    Zsig.fill(0.0);
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    for(int i = 0; i < 2*n_aug_+1; i++) {
        float px = Xsig_pred_(0,i);
        float py = Xsig_pred_(1,i);
        float v = Xsig_pred_(2,i);
        float theta = Xsig_pred_(3,i);

        Zsig(0,i) = sqrt(px*px + py*py);
        Zsig(1,i) = atan2(py, px);
        Zsig(2,i) = v*(px*cos(theta) + py*sin(theta))/Zsig(0,i);

        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    for(int i = 0; i < 2*n_aug_+1; i++) {
        VectorXd T =  Zsig.col(i) - z_pred;
        while(T(1) < -M_PI) T(1) += 2*M_PI;
        while(T(1) > M_PI)  T(1) -= 2*M_PI;

        S = S + weights_(i) * T * T.transpose();
    }

    MatrixXd R = MatrixXd(n_z, n_z);
    R(0,0) = std_radr_ * std_radr_;
    R(1,1) = std_radphi_ * std_radphi_;
    R(2,2) = std_radrd_ * std_radrd_;

    S = S + R;

    *S_out = S;
    *z_out = z_pred;
    *Zsig_out = Zsig;

    return;
}

void UKF::PredictLaserMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {
    lambda_ = 3 - n_aug_;
    int n_z = 2; //only px and py

    weights_(0) = lambda_/(lambda_ + n_aug_);
    for(int i = 1; i < 2*n_aug_+1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
    Zsig.fill(0.0);
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    for(int i = 0; i < 2*n_aug_+1; i++) {
        float px = Xsig_pred_(0,i);
        float py = Xsig_pred_(1,i);

        Zsig(0,i) = px;
        Zsig(1,i) = py;

        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    for(int i = 0; i < 2*n_aug_+1; i++) {
        VectorXd T =  Zsig.col(i) - z_pred;
        S = S + weights_(i) * T * T.transpose();
    }

    MatrixXd R = MatrixXd(n_z, n_z);
    R(0,0) = std_laspx_ * std_laspx_;
    R(1,1) = std_laspy_ * std_laspy_;

    S = S + R;

    *S_out = S;
    *z_out = z_pred;
    *Zsig_out = Zsig;

    return;

}

void UKF::UpdateRadarState(VectorXd& z_in, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {
    lambda_ = 3 - n_aug_;

    //weights_ = VectorXd(2*n_aug_+1);
    weights_(0) = lambda_/(lambda_ + n_aug_);
    for(int i = 1; i < 2*n_aug_+1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    int n_z = 3;
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    VectorXd z = VectorXd(n_z);
    z(0) = z_in(0);
    z(1) = z_in(1);
    z(2) = z_in(2);

    for(int i = 0; i < 2*n_aug_+1; i++) {
        VectorXd Tc0 = Xsig_pred_.col(i) - x_;
        while(Tc0(3) < -M_PI) Tc0(3) += 2*M_PI;
        while(Tc0(3) > M_PI)  Tc0(3) -= 2*M_PI;

        VectorXd Tc1 = Zsig.col(i) - z_pred;
        while(Tc1(1) < -M_PI) Tc1(1) += 2*M_PI;
        while(Tc1(1) > M_PI)  Tc1(1) -= 2*M_PI;

        Tc = Tc + weights_(i) * Tc0 * Tc1.transpose();
    }

    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;
    while(z_diff(1) < -M_PI) z_diff(1) += 2*M_PI;
    while(z_diff(1) > M_PI)  z_diff(1) -= 2*M_PI;

    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    //update radar NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

    return;
}


void UKF::UpdateLaserState(VectorXd& z_in, MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {
    lambda_ = 3 - n_aug_;

    weights_(0) = lambda_/(lambda_ + n_aug_);
    for(int i = 1; i < 2*n_aug_+1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

    int n_z = 2;
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    VectorXd z = VectorXd(n_z);
    z(0) = z_in(0);  //px
    z(1) = z_in(1);  //py

    for(int i = 0; i < 2*n_aug_+1; i++) {
        VectorXd Tc0 = Xsig_pred_.col(i) - x_;
        while(Tc0(3) < -M_PI) Tc0(3) += 2*M_PI;
        while(Tc0(3) > M_PI)  Tc0(3) -= 2*M_PI;

        VectorXd Tc1 = Zsig.col(i) - z_pred;

        Tc = Tc + weights_(i) * Tc0 * Tc1.transpose();
    }

    MatrixXd K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;

    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    //update lidar NIS
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

    return;
}

void UKF::outputNIS(double NIS) {

    ofstream outfile("./UKF_NIS.txt", ios::app);
    if (!outfile.is_open()) {

        std::cout << "output file can not open!" << endl;

        return ;
    }

    outfile << NIS << "\n";
    outfile.close();

    return;
}
