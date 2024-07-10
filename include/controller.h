#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

// #include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "zmq.hpp"
#include "robotmodel.h"
#include "trajectory.h"
#include "custommath.h"
// #include "cuda.cuh"
#include <signal.h>
#include <unistd.h>
#include <random>

// #include "mppi.h"
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

#define NECS2SEC 1000000000

class CController
{

public:
    CController();
    virtual ~CController();	

    // void read(double time, double* q, double* qdot, int check, double *trans_mat_goal);
    void read(double time, double* q, double* qdot, int check);
    void control_mujoco();
    void write(double* torque);
    // void write(float* Buffer);
    // double estimate_LR(double _input[]);
    double estimate_LR(double _input[]);
    double _output1;
    void test_thread();
    VectorXd _q, _qdot, _q_order;
    VectorXd _q2, _qdot2, _q_order2;
    // VectorXd _q; // joint angle
	// VectorXd _qdot; // joint velocity


    // void read_pybind(double time, std::array<double,7> qpos, std::array<double, 7> qvel);
    // void read_pybind(double time, std::array<double,14> qpos, std::array<double, 14> qvel, std::array<double, 3> alpha);
    // void read_pybind(double time, std::array<double,7> qpos, std::array<double, 7> qvel, double alpha);
    // std::vector<double> write_pybind();
    // std::vector<double> state_for_pybind();
    std::vector<double> torque_command;
    std::vector<double> X_data_for_pybind;
    void Initialize();
    void reset_goal_pybind();

    float Buffer[14];
    // read_pybind(double t, std::array<double,15> q, std::array<double, 15> qdot, double alpha)



    
    float  _goal_pos_cuda[7], _returnArray[71], _q_for_cuda[7], _qdot_for_cuda[7], _trans_mat_goal[12];
    double tmp_pos_array[35], tmp_vel_array[35];
    double _alpha[1], _beta[1], _time_rl[1];

private:
    
    void ModelUpdate();
    void motionPlan();

    void reset_target(double motion_time, VectorXd target_joint_position, int robot);
    void reset_target(double motion_time, VectorXd target_joint_position, VectorXd target_joint_velocity);
    void reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori);

    double generateRandomReal(double a, double b);
    // void readJointState();
    int _cnt, tmp_signal;
	// tmp_signal


    
    
    VectorXd _torque, _pre_q, _pre_qdot, _pre_qdotdot_des; // joint torque
    VectorXd tmp_q;
    
    VectorXd _torque2, _pre_q2, _pre_qdot2, _pre_qdotdot_des2; // joint torque
    VectorXd tmp_q2;

    int _k; // DOF

    bool _bool_init;
    double _t;
    double _dt;
	double _init_t;
	double _pre_t;

    //controller
	double _kpj, _kdj; //joint P,D gain
    double _x_kp; // task control P gain

    void JointControl();
    void JointControl2();
    void CLIK();

    // robotmodel
    CModel Model;
    CModel Model2;
    // CModel2 Model2;
    
    int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

    int _control_mode; //1: joint space, 2: operational space
    VectorXd _q_home; // joint home position

    //motion trajectory
	double _start_time, _end_time, _motion_time;

    CTrajectory JointTrajectory; // joint space trajectory
    CTrajectory JointTrajectory2; // joint space trajectory
    
    HTrajectory HandTrajectory; // task space trajectory

    bool _bool_joint_motion, _bool_ee_motion; // motion check

    VectorXd _q_des, _qdot_des; 
    VectorXd _q_goal, _qdot_goal;
    VectorXd _x_des_hand, _xdot_des_hand;
    VectorXd _x_goal_hand, _xdot_goal_hand;
    Vector3d _pos_goal_hand, _rpy_goal_hand;

    
    MatrixXd _A_diagonal; // diagonal inertia matrix
    MatrixXd _J_hands; // jacobian matrix
    MatrixXd _J_bar_hands; // pseudo invere jacobian matrix

    VectorXd _x_hand, _xdot_hand; // End-effector


    VectorXd _x_err_hand;
    Matrix3d _R_des_hand;

    /// Robot 2  //
    VectorXd _q_des2, _qdot_des2; 
    VectorXd _q_goal2, _qdot_goal2;
    VectorXd _x_des_hand2, _xdot_des_hand2;
    VectorXd _x_goal_hand2, _xdot_goal_hand2;
    Vector3d _pos_goal_hand2, _rpy_goal_hand2;

    MatrixXd _A_diagonal2; // diagonal inertia matrix
    MatrixXd _J_hands2; // jacobian matrix
    MatrixXd _J_bar_hands2; // pseudo invere jacobian matrix

    VectorXd _x_hand2, _xdot_hand2; // End-effector


    VectorXd _x_err_hand2;
    Matrix3d _R_des_hand2;



    MatrixXd _I; // Identity matrix

    ////////////////////////////////////////
    MatrixXd txt_position, txt_velocity;
    VectorXd txt_nsecs;
    int _size;
    ////////////////////////////////////////
    float rand_FloatRange(float a, float b);
    ////////////////////save_stack/////////////////
    void save_stack();
    bool check_save_log_once, save_q_once, save_q_once1, save_q_once2;
    ostringstream filename;
    ifstream weight0;
    ofstream output;
    VectorXd _min_joint_position, _max_joint_position, _q_home_old, _q_acc, _old_torque, _mppi, _qdotdot_des;
    VectorXd _min_joint_velocity, _max_joint_velocity;
    VectorXd _q_home_old2, _q_acc2;
    
    
    MatrixXd _q_acc_matrix;
    double log_output[2000000], prevent_duplicate, _tmp;
    int check_7, a , cnt2, _rat, _ratt, ratrat, timestep_cnt, _cnt_mppi;
    ///////////////////////////////load weight////////////////
    void load_weight();
    int _input_size;
    // double _input[14]; // input node
	// double _hidden0[256] = {}; // hidden layer 1
	// double _hidden1[256] = {}; // hidden layer 2	
	// double _hidden2[128] = {}; // hidden layer 2	
	// double _weight0[14][256] = {{0.0}}; // weight 1
	// double _weight1[256] = {};   // bias 1
	// double _weight2[256][256] = {{0.0}}; // weight 2
	// double _weight3[256] = {};   // bias 2
	// double _weight4[256][128] = {{0.0}}; // weight 3
	// double _weight5[128] = {};   // bias 3
    // double _weight6[128] = {}; // weight 4
	// double _weight7 = 0;   // bias 4
    
    // test_heavy_0419_deep (성능 collision o 예측 : 87%, collision x 예측 : 84%)
    //                       2천만개 이상 sample 비교 누적 한 결과
    // double _hidden0[256]; // hidden layer 1
	// double _hidden1[256]; // hidden layer 2	
	// double _hidden2[128]; // hidden layer 2	
	// double _weight0[14][256]; // weight 1
	// double _weight1[256];   // bias 1
	// double _weight2[256][256]; // weight 2
	// double _weight3[256];   // bias 2
	// double _weight4[256][128]; // weight 3
	// double _weight5[128];   // bias 3
    // double _weight6[128]; // weight 4
	// double _weight7;   // bias 4

    // test_heavy_0419_dropout(성능 collision o 예측 : 90%, collision x 예측 : 83%)
    // double _hidden0[1024]; // hidden layer 1
	// double _hidden1[1024]; // hidden layer 2	
	// double _hidden2[64]; // hidden layer 2	
	// double _weight0[14][1024]; // weight 1
	// double _weight1[1024];   // bias 1
	// double _weight2[1024][1024]; // weight 2
	// double _weight3[1024];   // bias 2
	// double _weight4[1024][64]; // weight 3
	// double _weight5[64];   // bias 3
    // double _weight6[64]; // weight 4
	// double _weight7;   // bias 4

    // test_heavy_0422_deep2 (성능 collision o 예측 : 86%, collision x 예측 : 85%)
    // double _hidden0[256]; // hidden layer 1
	// double _hidden1[256]; // hidden layer 2	
	// double _hidden2[256]; // hidden layer 2	
	// double _hidden3[128]; // hidden layer 2	
	// double _weight0[14][256]; // weight 1
	// double _weight1[256];   // bias 1
	// double _weight2[256][256]; // weight 2
	// double _weight3[256];   // bias 2
	// double _weight4[256][256]; // weight 2
	// double _weight5[256];   // bias 2
    // double _weight6[256][128]; // weight 3
	// double _weight7[128];   // bias 3
    // double _weight8[128]; // weight 4
	// double _weight9;   // bias 4

double _input[28]; // input node
    // qqd_503 (성능 collision o 예측 : 90%, collision x 예측 : 86%)
    double _hidden0[1024]; // hidden layer 1
	double _hidden1[1024]; // hidden layer 2	
	double _hidden2[64]; // hidden layer 2	
	double _weight0[28][1024]; // weight 1
	double _weight1[1024];   // bias 1
	double _weight2[1024][1024]; // weight 2
	double _weight3[1024];   // bias 2
	double _weight4[1024][64]; // weight 2
	double _weight5[64];   // bias 2
    double _weight6[64]; // weight 4
	double _weight7;   // bias 4
    
    int _nodes0,_nodes1,_nodes2,_nodes3, tempcheck, NconOCollX, diff_all, clock, _nconminuscnt, _nconcnt, NconXCollO;
    bool nocheck;
    void test_LR(int check);

    double sel_;

    std::uniform_real_distribution<double> random_motion_time;

// Declare rng as extern
    std::mt19937 rng;
    // int time_cnt;
    // double time_check, max_time_check[500];
    // int _nconminuscnt, _nconcnt, NconXCollO;

};

#endif