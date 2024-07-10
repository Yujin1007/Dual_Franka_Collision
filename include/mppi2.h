#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "controller.h"

#include <chrono>
#include <random>

using namespace std;
using namespace Eigen;

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

private:

	void Initialize();
	void Initialize_for_MPPI();

	void generate_random(int _sampling);
	void cost_function();
	void update_state(int _sampling);

	///////////관절별////////////////
	// void generate_random(int joint_num);
	// void cost_function(int joint_num);
	// void update_state(int joint_num);
	///////////관절별////////////////

	double _cost_sum, _NN_output, _dt, dT, _alpha, _beta, _random_matrix[7][300][100];
	MatrixXd _jointstate_matrix, _random_input, lower_triangular_matrix_7, upper_triangular_matrix_7, _pos_state, _vel_state, _acc_state, _cost;
	VectorXd _cost1, _cost2, _min_joint_position, _max_joint_position, _q_goal, _qdot_init, _q_init, weight_sum, _min_cost;
	VectorXd _next_acc_state, _A, _B, _weighted_A, _weighted_B, _temp_vec, _q_des, _qdot_des, _cost_joint;
	int _sampling_num, _time_window, _save_best_num;
};