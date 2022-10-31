#pragma once
#include <iostream>
#include <Eigen/Dense>

namespace evsts{
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t MEASUREMENT_DIM>
class BayesianFilter{
    public:
        Eigen::Matrix<float,STATE_DIM,1> state;
        virtual void predict(const Eigen::Matrix<float,CONTROL_DIM,1>& u) = 0;
        virtual void update(const Eigen::Matrix<float,MEASUREMENT_DIM,1>& z) = 0;
};

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t MEASUREMENT_DIM>
class KalmanFilter: public BayesianFilter<STATE_DIM, CONTROL_DIM, MEASUREMENT_DIM>{    
    Eigen::Matrix<float,STATE_DIM,STATE_DIM> F; // state transition matrix x(t+1) = F*x(t) + P 
    Eigen::Matrix<float,STATE_DIM,STATE_DIM> P; // state uncertainty 
    Eigen::Matrix<float,MEASUREMENT_DIM,STATE_DIM> H; // measurement matrix z(t+1) = H*z(t) + R
    Eigen::Matrix<float,MEASUREMENT_DIM,MEASUREMENT_DIM> R; // measurement uncertainty

    public:
        KalmanFilter(const Eigen::Matrix<float,STATE_DIM,STATE_DIM>& F_, 
        const Eigen::Matrix<float,STATE_DIM,STATE_DIM>& P_, 
        const Eigen::Matrix<float,MEASUREMENT_DIM,STATE_DIM>& H_,
        Eigen::Matrix<float,MEASUREMENT_DIM,MEASUREMENT_DIM>& R_):F(F_), P(P_), H(H_), R(R_) {}

        KalmanFilter(const float dt){
            F << 1, 0, dt, 0,
                 0, 1, 0, dt,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
            P << 0, 0, 0, 0,
                 0, 0, 0, 0,
                 0, 0, 1000, 0,
                 0, 0, 0, 1000;
            H << 1, 0, 0, 0,
                 0, 1, 0, 0;
            R << 0.1, 0,
                 0, 0.1;
        }

        void set_initial_state(const Eigen::Matrix<float,STATE_DIM,1>& s){
            this->state = s;
        }

        void predict(const Eigen::Matrix<float,CONTROL_DIM,1>& u){
            Eigen::Matrix<float,STATE_DIM,CONTROL_DIM> B;
            B.setZero();
            this->state = F * this->state + B*u;
            P = F * P * F.transpose();
        }

        void update(const Eigen::Matrix<float,MEASUREMENT_DIM,1>& z){
            Eigen::MatrixXf y = z - (H * this->state);
            Eigen::MatrixXf S = H * P * H.transpose() + R;
            Eigen::MatrixXf K = P * H.transpose() * S.inverse();
            this->state = this->state + (K * y);
            P = (Eigen::Matrix<float,STATE_DIM,STATE_DIM>::Identity() - (K * H)) * P;
        }
};
}