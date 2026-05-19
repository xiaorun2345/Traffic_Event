#include <math.h>
#include <iostream>
#include "N_Kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace camera 
{

const float PI2 = 2 * M_PI;

N_KalmanFilter::N_KalmanFilter() 
{
    // initializing matrices
    R_ = Eigen::MatrixXd(2, 2);

    //measurement covariance matrix - laser
    R_ << 0.0225, 0,
            0, 0.0225;

    H_ = Eigen::MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // initialize the kalman filter variables
    P_ = Eigen::MatrixXd(4, 4);
    // P_ << 1, 0, 0, 0,
    //       0, 1, 0, 0,
    //       0, 0, 1000, 0,
    //       0, 0, 0, 1000;
    P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 10, 0,
        0, 0, 0, 10;


    F_ = Eigen::MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    Q_ = MatrixXd(4, 4);
}

N_KalmanFilter::N_KalmanFilter(int dim) 
{
    int dim2 = dim * 2;
    // initializing matrices
    // R_ = Eigen::MatrixXd(dim, dim);
    R_ = Eigen::MatrixXd::Identity(dim, dim);  //单位矩阵

    //measurement covariance matrix
    // R_ << 0.0225, 0,
    //       0, 0.0225;
    for(int i = 0; i < dim; i++) 
    {
        R_(i, i) = 0.0225;
    }

    H_ = Eigen::MatrixXd::Identity(dim, dim2);
    // H_ << 1, 0, 0, 0,
    //       0, 1, 0, 0;

    // initialize the kalman filter variables
    P_ = Eigen::MatrixXd::Identity(dim2, dim2);
    // P_ << 1, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, 10, 0,
    //     0, 0, 0, 10;

    F_ = Eigen::MatrixXd::Identity(dim2, dim2);
    // F_ << 1, 0, 1, 0,
    //       0, 1, 0, 1,
    //       0, 0, 1, 0,
    //       0, 0, 0, 1;
    
    for(int i = 0; i < dim; i++) 
    {
        F_(i, i+dim) = 1.f;
    }

    Q_ = Eigen::MatrixXd(dim2, dim2);
}


N_KalmanFilter::~N_KalmanFilter() {}

void N_KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void N_KalmanFilter::SetFMatrix(float dt) 
{
    F_(0, 2) = dt;
    F_(1, 3) = dt;
}

void N_KalmanFilter::SetFMatrix(float dt, int dim) 
{
    // F_(0, 2) = dt;
    // F_(1, 3) = dt;
    for(int i=0; i < dim; i++)
    {
        F_(i, i+dim) = dt;
    }
}

void N_KalmanFilter::SetProcessCovarianceNoiseSquare(float dt, float noise_ax, float noise_ay)
{
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
            0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
            dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
            0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;
}

void N_KalmanFilter::SetProcessCovarianceNoiseSquare(float dt, int dim, float noise)
{
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    int dim_2 = dim * 2;
    for(int i=0;i<dim_2;i++)
        Q_(i,i) = dt * 0.1*x_(i);

    // for(int i=0; i<dim; i++)
    // {
    //   Q_(i, i+dim) = dt*noise;
    //   // Q_(i+dim, i) = dt_3/3*noise;
    // }
}

void N_KalmanFilter::Predict() 
{
    /**
     TODO:
        * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void N_KalmanFilter::Update(const VectorXd &z) 
{
    /**
     TODO:
        * update the state by using Kalman Filter equations
    */

    VectorXd z_pred = H_ * x_;

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;

    // std::cout << "The determinant of S is " << S.determinant() << std::endl;
    // MatrixXd Si = S.inverse();
    MatrixXd II = Eigen::MatrixXd::Identity(S.rows(), S.cols());
    MatrixXd Si = S.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(II);

    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

VectorXd RadarCartesianToPolar(const VectorXd &x_state)
{
    /*
    * convert radar measurements from cartesian coordinates (x, y, vx, vy) to
    * polar (rho, phi, rho_dot) coordinates
    */
    float px, py, vx, vy;
    px = x_state[0];
    py = x_state[1];
    vx = x_state[2];
    vy = x_state[3];

    float rho, phi, rho_dot;
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);  // returns values between -pi and pi

    // if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
    if(rho < 0.000001)
        rho = 0.000001;

    rho_dot = (px * vx + py * vy) / rho;

    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;

    return z_pred;
}

void N_KalmanFilter::UpdateEKF(const VectorXd &z) 
{
    /**
        * update the state by using Extended Kalman Filter equations
    */

    // convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
    VectorXd z_pred = RadarCartesianToPolar(x_);
    VectorXd y = z - z_pred;

    // normalize the angle between -pi to pi
    while(y(1) > M_PI)
    {
        y(1) -= PI2;
    }

    while(y(1) < -M_PI)
    {
        y(1) += PI2;
    }

    // following is exact the same as in the function of N_KalmanFilter::Update()
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void N_KalmanFilter::SetMeasurementNoiseSquare(double std_x, double std_y)
{
    R_ << 
        std_x * std_x, 0,
        0, std_y* std_y;
}

void N_KalmanFilter::SetMeasurementNoiseSquare(int dim, double std_x, double std_y)
{
    for(int i=0; i< dim; i++)
    {
        // R_(i,i) = std_x * std_y;
        R_(i, i) = 0.1 * x_(i);
    }
}

} //namespace camera