#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "controller.h"

#include <chrono>
#include <random>

using namespace std;
using namespace Eigen;
// #define N 64*7*32


class CMppi
{

public:
	CMppi();
	virtual ~CMppi();
	MatrixXd MPPI_mat(VectorXd q_init, VectorXd qdot_init);
	VectorXd MPPI_vec(VectorXd q_init, VectorXd qdot_init);
	void Trajectory_init(VectorXd q_goal);
	MatrixXd _select_input_matrix, _temp_imput;
	VectorXd _select_input;
	double _NN_input[7]; // input node
	// double  _goal_pos_cuda[7], _init_pos_cuda[7], _init_vel_cuda[7], _returnArray[21];
	

private:

	void Initialize();
	void Initialize_for_MPPI();

	// void generate_random();
	void update_state(int sam_num);
	void cost_function(int joint_num);
	

	///////////관절별////////////////
	// void generate_random(int joint_num);
	// void cost_function(int joint_num);
	// void update_state(int joint_num);
	///////////관절별////////////////

	double _cost_sum, _NN_output, _dt, dT, _alpha, _beta, _random_matrix[64][7][32], _mean_matrix[64][7][32];
	double  _min_cost, _cost1, weight_sum, _cost2, position_sec1, position_sec2, velocity_lim, _cost_self_col, cost_pos, cost_vel, cost_acc;
	MatrixXd _jointstate_matrix, _random_input, lower_triangular_matrix_7, upper_triangular_matrix_7, _pos_state, _vel_state, _acc_state, _cost_j;
	VectorXd _min_joint_position, _max_joint_position, _q_goal, _qdot_init, _q_init, _cost, _min_cost_vec, weight_sum_vec;
	VectorXd _next_acc_state, _A, _B, _weighted_A, _weighted_B, _temp_vec, _q_des, _qdot_des, _cost_joint, check_goal;
	// int 1 _save_best_num;
};