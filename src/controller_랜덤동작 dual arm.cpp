// controller *dual arm collision dataset 생성 코드
#include "controller.h"
#include <chrono>

// #include <pybind11/operators.h>
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>

#include <fstream> // ifstream header
#include <iostream>
#include <string> // getline header
#include <random>

CController::CController()
{
	_k = 7;
	Initialize();

	// readJointState();
	// _bool_plan.setZero(_size + 10);
	// _time_plan.resize(_size + 10);
	// _time_plan.setConstant(5.0);
}

CController::~CController()
{
}

// void CController::readJointState()
// {
// 	std::ifstream file("/home/kist-robot2/plan1.txt");

// 	file >> _size;

// 	txt_nsecs.setZero(_size);
// 	txt_position.setZero(_size,7);
// 	txt_velocity.setZero(_size,7);

// 	for(int i = 0; i < _size; i++)
// 	{
// 		file >> txt_nsecs(i);

// 		for(int j = 0; j < 7; j++)
// 		{
// 			file >> txt_position(i,j);
// 			file >> txt_velocity(i,j);
// 		}
// 	}

// 	for(int i = 0; i < _size; i++)
// 	{
// 		std::cout << "i : " << i << txt_nsecs(i) << std::endl;

// 		for(int j = 0; j < 7; j++)
// 		{
// 			std::cout << txt_position(i,j) << " ";
// 		}
// 		std::cout << std::endl;
// 		for(int j = 0; j < 7; j++)
// 		{
// 			std::cout << txt_velocity(i,j) << " ";
// 		}
// 		std::cout << std::endl;
// 	}

// 	std::cout << "Load Finish -> size : " << _size << std::endl;

// 	file.close();
// }

void CController::read(double t, double *q, double *qdot, int check)
{
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i];
	}

	for (int i = 0; i < _k; i++)
	{
		_q2(i) = q[i + _k];
		_qdot2(i) = qdot[i + _k];
	}

	// cout<<check<<endl;

	save_q_once = true;
	save_q_once1 = true;
	save_q_once2 = true;

	if (abs(_q_home_old(0) - _q(0)) <= prevent_duplicate)
	{
		if (abs(_q_home_old(1) - _q(1)) <= prevent_duplicate)
		{
			if (abs(_q_home_old(2) - _q(2)) <= prevent_duplicate)
			{
				if (abs(_q_home_old(3) - _q(3)) <= prevent_duplicate)
				{
					if (abs(_q_home_old(4) - _q(4)) <= prevent_duplicate)
					{
						if (abs(_q_home_old(5) - _q(5)) <= prevent_duplicate)
						{
							if (abs(_q_home_old(6) - _q(6)) <= prevent_duplicate)
							{
								save_q_once1 = false;
							}
						}
					}
				}
			}
		}
	}

	if (abs(_q_home_old2(0) - _q2(0)) <= prevent_duplicate)
	{
		if (abs(_q_home_old2(1) - _q2(1)) <= prevent_duplicate)
		{
			if (abs(_q_home_old2(2) - _q2(2)) <= prevent_duplicate)
			{
				if (abs(_q_home_old2(3) - _q2(3)) <= prevent_duplicate)
				{
					if (abs(_q_home_old2(4) - _q2(4)) <= prevent_duplicate)
					{
						if (abs(_q_home_old2(5) - _q2(5)) <= prevent_duplicate)
						{
							if (abs(_q_home_old2(6) - _q2(6)) <= prevent_duplicate)
							{
								save_q_once2 = false;
							}
						}
					}
				}
			}
		}
	}
	if ((save_q_once1 && save_q_once2) == false)
	{
		save_q_once = false;
	}

	// if(check >= 1)
	// {
	// 	if(save_q_once == true)
	// 	{
	// 		for(int i = 0; i< 7 ; i++)
	// 		{
	// 			log_output[a] = (_q(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
	// 			if(log_output[a] > 1)
	// 			{
	// 				log_output[a] = 1;
	// 			}
	// 			else if(log_output[a] < -1)
	// 			{
	// 				log_output[a] = -1;
	// 			}
	// 			_q_home_old(i) = _q(i);
	// 			a++;
	// 		}for(int i = 0; i< 7 ; i++)
	// 		{
	// 			log_output[a+_k] = (_q2(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
	// 			if(log_output[a+_k] > 1)
	// 			{
	// 				log_output[a+_k] = 1;
	// 			}
	// 			else if(log_output[a+_k] < -1)
	// 			{
	// 				log_output[a+_k] = -1;
	// 			}
	// 			_q_home_old2(i) = _q2(i);
	// 			a++;
	// 		}
	// 		log_output[a] = 1;
	// 		a++;
	// 		_rat++;
	// 	}

	// }
	// else
	// {
	// 	cnt2++;
	// 	if(cnt2 >= ratrat)
	// 	{
	// 		for(int i = 0; i< 7 ; i++)
	// 		{
	// 			log_output[a] = (_q(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
	// 			if(log_output[a] > 1)
	// 			{
	// 				log_output[a] = 1;
	// 			}
	// 			else if(log_output[a] < -1)
	// 			{
	// 				log_output[a] = -1;
	// 			}
	// 			a++;
	// 		}for(int i = 0; i< 7 ; i++)
	// 		{
	// 			log_output[a+_k] = (_q2(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
	// 			if(log_output[a] > 1)
	// 			{
	// 				log_output[a+_k] = 1;
	// 			}
	// 			else if(log_output[a+_k] < -1)
	// 			{
	// 				log_output[a+_k] = -1;
	// 			}
	// 			a++;
	// 		}
	// 		log_output[a] = 0;
	// 		a++;
	// 		cnt2 = 0;
	// 		_ratt++;
	// 	}
	// }
	if (save_q_once == true)
	{

		// cout<<((_q - _min_joint_position) / (abs(_max_joint_position - _min_joint_position) / 2) - 1).transpose()<<" "<<((_q2 - _min_joint_position) / (abs(_max_joint_position - _min_joint_position) / 2) - 1).transpose()<<endl;
		// cout<<((_q - _min_joint_position) / (abs(_max_joint_position - _min_joint_position) / 2) - 1).transpose()<<" "<<((_q2 - _min_joint_position) / (abs(_max_joint_position - _min_joint_position) / 2) - 1).transpose()<<endl;

		if (check >= 1)
		{
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_q(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				_q_home_old(i) = _q(i);
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_q2(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				_q_home_old2(i) = _q2(i);
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_qdot(i) - _min_joint_velocity(i)) / (abs(_max_joint_velocity(i) - _min_joint_velocity(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_qdot2(i) - _min_joint_velocity(i)) / (abs(_max_joint_velocity(i) - _min_joint_velocity(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				a++;
			}

			log_output[a] = 1;
			a++;
			_rat++;
		}
		else
		{

			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_q(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				_q_home_old(i) = _q(i);
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_q2(i) - _min_joint_position(i)) / (abs(_max_joint_position(i) - _min_joint_position(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				_q_home_old2(i) = _q2(i);
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_qdot(i) - _min_joint_velocity(i)) / (abs(_max_joint_velocity(i) - _min_joint_velocity(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				a++;
			}
			for (int i = 0; i < 7; i++)
			{
				log_output[a] = (_qdot2(i) - _min_joint_velocity(i)) / (abs(_max_joint_velocity(i) - _min_joint_velocity(i)) / 2) - 1;
				if (log_output[a] > 1)
				{
					log_output[a] = 1;
				}
				else if (log_output[a] < -1)
				{
					log_output[a] = -1;
				}
				a++;
			}

			log_output[a] = 0;
			a++;
			_ratt++;
		}
	}
}

void CController::write(double *torque)
{
	for (int i = 0; i < _k; i++)
	{
		torque[i] = _torque(i);
		// torque[i] = 0;
	}
	for (int i = 0; i < _k; i++)
	{
		torque[i + _k] = _torque2(i);
	}
	// cout<<"torque:"<<_torque.transpose()<<endl;
}

// for pybind11
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// void CController::read_pybind(double t, std::array<double,15> q, std::array<double, 15> qdot)
// {
// 	_t = t;
// 	if (_bool_init == true)
// 	{
// 		_init_t = _t;
// 		_bool_init = false;
// 	}

// 	_dt = t - _pre_t;
// 	_pre_t = t;

// 	for (int i = 0; i < _k; i++)
// 	{
// 		_q(i) = q[i];
// 		_qdot(i) = qdot[i];
// 	}
// }

// std::vector<double> CController::write_pybind()
// {
// 	torque_command.clear();

// 	for (int i = 0; i < _k; i++)
// 	{
// 		torque_command.push_back(_torque(i));
// 	}

// 	return torque_command;
// }
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void CController::control_mujoco()
{
	ModelUpdate();
	motionPlan();
	if (_control_mode == 1) // joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);

			JointTrajectory2.reset_initial(_start_time, _q2, _qdot2);
			JointTrajectory2.update_goal(_q_goal2, _qdot_goal2, _end_time);
			// JointTrajectory2.reset_initial(_start_time, _q2, _qdot2);
			// JointTrajectory2.update_goal(_q_goal2, _qdot_goal2, _end_time);
			// cout<<"qd1 init:"<<_qdot.transpose()<<endl;
			// cout<<"qd2 init:"<<_qdot2.transpose()<<endl;
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		JointTrajectory2.update_time(_t);
		_q_des2 = JointTrajectory2.position_cubicSpline();
		_qdot_des2 = JointTrajectory2.velocity_cubicSpline();

		JointControl();
		JointControl2();

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
			// cout<<"q 1 now :"<<_q.transpose()<<endl;
			// cout<<"q 2 now :"<<_q2.transpose()<<endl;
		}
	}
	else if (_control_mode == 2)
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time);
			_bool_ee_motion = true;
		}

		HandTrajectory.update_time(_t);

		_x_des_hand.head(3) = HandTrajectory.position_cubicSpline();
		_R_des_hand = HandTrajectory.rotationCubic();
		_x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);

		_xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
		_xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();

		if (HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
		CLIK();
	}

	// _torque = _q_des;
}

void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;

	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);

	_xdot_hand = Model._xdot_hand;

	/// hand 2 ////

	Model2.update_kinematics(_q2, _qdot2);
	Model2.update_dynamics();
	Model2.calculate_EE_Jacobians();
	Model2.calculate_EE_positions_orientations();
	Model2.calculate_EE_velocity();

	_J_hands2 = Model2._J_hand;

	_x_hand2.head(3) = Model2._x_hand;

	_x_hand2.tail(3) = CustomMath::GetBodyRotationAngle(Model2._R_hand);

	_xdot_hand2 = Model2._xdot_hand;

	// cout<<"Model 1 jacobian \n"<<_J_hands.transpose()<<endl;
	// cout<<"Model 2 jacobian \n"<<_J_hands2.transpose()<<endl;
}

void CController::motionPlan()
{
	if (_bool_plan(_cnt_plan) == 1)
	{
		// cout<<"_cnt_plan : "<<_cnt_plan<<endl;
		// if(_cnt_plan == timestep_cnt)
		// {

		// }
		// else
		// {
		if (a >= 19999990)
		{
			// reset_target(50, _q);
			// cout<<"here??"<<endl;
			save_stack();
			exit(0);
			cout << "end" << endl;
			// _cnt_plan++;
		}
		else
		{
			_cnt_plan++;
			if (_cnt_plan % 60 == 0)
			{
				cout << "_cnt_plan : " << _cnt_plan << endl;
				save_stack();
				cout << "ratio(collision) : " << (double)_rat * 100 / (_ratt + _rat) << "%" << endl;
				// cout << "ratrat : " << ratrat << endl;

				// _tmp = (double)_ratt * 100 / (_ratt + _rat);
				// if (_tmp <= 65 && _tmp > 52)
				// {
				// 	ratrat = ratrat - 1;
				// }
				// else if (_tmp <= 75 && _tmp > 65)
				// {
				// 	ratrat = ratrat - 2;
				// }
				// else if (_tmp > 75)
				// {
				// 	ratrat = ratrat - 3;
				// }
				// else if (_tmp < 48 && _tmp >= 35)
				// {
				// 	ratrat = ratrat + 1;
				// }
				// else if (_tmp < 35 && _tmp >= 25)
				// {
				// 	ratrat = ratrat + 2;
				// }
				// else if (_tmp < 25)
				// {
				// 	ratrat = ratrat + 3;
				// }
				// if (ratrat >= 300)
				// {
				// 	ratrat = 300;
				// }
				// else if (ratrat <= 0)
				// {
				// 	ratrat = 0;
				// }
			}

			_q_order.setZero(7);
			_q_order2.setZero(7);
			for (int i = 0; i < 7; i++)
			{
				// _q_order(i) =rand_FloatRange(_min_joint_position(i),_max_joint_position(i));
				_q_order(i) = generateRandomReal(_min_joint_position(i), _max_joint_position(i));
				_q_order2(i) = generateRandomReal(_min_joint_position(i), _max_joint_position(i));
			}
			// _q_order = _q;
			// _q_order2 = _q;

			// reset_target(10.0, _q_home, 1);
			// reset_target(10.0, _q_home, 2);
			double random_time = random_motion_time(rng)*5.0 + 1.0; //[1,6]
			reset_target(random_time, _q_order, 1);
			reset_target(random_time, _q_order2, 2);
			// cout<<"random time :"<<random_time<<endl;
			// _q_order2.setZero(7);
			// for(int i = 0; i< 7; i++)
			// {
			// 	_q_order2(i) =rand_FloatRange(_min_joint_position(i),_max_joint_position(i));
			// }
			// reset_target(1.0, _q_order2);
		}
	}
	// }
	///////////////////////////////////기존 코드/////////////////////////////////////////////
	// _time_plan(1) = 2.0; // move home position
	// _time_plan(2) = 1.0; // wait
	// _time_plan(3) = 2.0; // joint goal motion
	// _time_plan(4) = 1.0; // wait
	// _time_plan(5) = 2.0; // task goal motion
	// _time_plan(6) = 100000.0; // wait

	// if (_bool_plan(_cnt_plan) == 1)
	// {
	// 	_cnt_plan = _cnt_plan + 1;
	// 	// for(int i = 0; i < 6; i++){
	// 	// 	cout << _x_hand(i) << "  ";
	// 	// }
	// 	// std::cout << "cnt : " << _cnt_plan << std::endl;
	// 	// cout << endl;

	// 	if(_cnt_plan == 1)
	// 	{
	// 		// _q_home = txt_position.block<1, 7>(0, 0);
	// 		reset_target(_time_plan(_cnt_plan), _q_home);
	// 	}
	// 	else if (_cnt_plan == 2)
	// 	{
	// 		reset_target(_time_plan(_cnt_plan), _q);
	// 	}
	// 	else if (_cnt_plan == 3)
	// 	{
	// 		_q_goal.setZero(_k);
	// 		_q_goal(0) = 0.0;
	// 		_q_goal(1) = 0.0;
	// 		_q_goal(2) = 0.0;
	// 		_q_goal(3) = -90.0 * DEG2RAD;
	// 		_q_goal(4) = 0.0;
	// 		_q_goal(5) = 90 * DEG2RAD;
	// 		_q_goal(6) = 0.0;

	// 		reset_target(_time_plan(_cnt_plan), _q_goal);
	// 	}
	// 	else if (_cnt_plan == 4)
	// 	{
	// 		reset_target(_time_plan(_cnt_plan), _q);
	// 	}
	// 	else if (_cnt_plan == 5)
	// 	{
	// 		_pos_goal_hand(0) = _x_hand(0) - 0.2;
	// 		_pos_goal_hand(1) = _x_hand(1) + 0.2;
	// 		_pos_goal_hand(2) = _x_hand(2);

	// 		_rpy_goal_hand(0) = _x_hand(3);
	// 		_rpy_goal_hand(1) = _x_hand(4) - 0.2;
	// 		_rpy_goal_hand(2) = _x_hand(5) + 0.5;

	// 		reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
	// 	}
	// 	else if (_cnt_plan == 6)
	// 	{
	// 		reset_target(_time_plan(_cnt_plan), _q);
	// 	}

	// 	// /////////////////////// txt data /////////////////////////////
	// 	// else if(_cnt_plan == 2)
	// 	// {
	// 	// 	_time_plan(_cnt_plan) = 5.0;
	// 	// 	reset_target(_time_plan(_cnt_plan), _q);
	// 	// }
	// 	// else if(_cnt_plan == (_size+2))
	// 	// {
	// 	// 	_time_plan(_cnt_plan) = 100000.0;
	// 	// 	reset_target(_time_plan(_cnt_plan), _q);
	// 	// }
	// 	// else
	// 	// {
	// 	// 	std::cout << "cnt : " << _cnt_plan << std::endl;
	// 	// 	_time_plan(_cnt_plan) = (txt_nsecs(_cnt_plan-2) - txt_nsecs(_cnt_plan-3))/NECS2SEC;
	// 	// 	_q_goal = txt_position.block<1, 7>(_cnt_plan-2, 0);
	// 	// 	_qdot_goal = txt_velocity.block<1, 7>(_cnt_plan-2, 0);
	// 	// 	reset_target(_time_plan(_cnt_plan), _q_goal, _qdot_goal);
	// 	// }
	// 	// /////////////////////////////////////////////////////////////

	// }
	///////////////////////////////////기존 코드/////////////////////////////////////////////
}

void CController::reset_target(double motion_time, VectorXd target_joint_position, int robot)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	if (robot == 1)
	{
		_q_goal = target_joint_position.head(7);
		_qdot_goal.setZero();
	}
	else if (robot == 2)
	{
		_q_goal2 = target_joint_position.head(7);
		_qdot_goal2.setZero();
	}
}

void CController::reset_target(double motion_time, VectorXd target_joint_position, VectorXd target_joint_velocity)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(7);
	_qdot_goal = target_joint_velocity.head(7);
}

void CController::reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;
	_xdot_goal_hand.setZero();
}

void CController::JointControl()
{
	_torque.setZero();
	for (int i = 0; i < 7; i++)
	{
		_A_diagonal(i, i) = Model._A(i, i);
	}
	// _torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot)) + Model._bg;
	_torque = _A_diagonal * (_kpj * (_q_des - _q) + _kdj * (_qdot_des - _qdot)) + Model._bgg;

	// cout<<"1 q des :"<<_q_des.transpose()<<endl;
	// cout<<"1 q now :"<<_q.transpose()<<endl;
	// cout<<"1 qd des :"<<_qdot_des.transpose()<<endl;
	// cout<<"1 qd now :"<<_qdot.transpose()<<endl;
	// cout<<"1 torque :"<<_torque.transpose()<<endl;
	// _torque = Model._bgg;
}
void CController::JointControl2()
{
	_torque2.setZero();
	for (int i = 0; i < 7; i++)
	{
		_A_diagonal2(i, i) = Model2._A(i, i);
	}
	_torque2 = _A_diagonal2 * (_kpj * (_q_des2 - _q2) + _kdj * (_qdot_des2 - _qdot2)) + Model2._bgg;
	// _torque2 = _A_diagonal2 * (_kpj * (_q_des - _q2) + _kdj * (_qdot_des - _qdot2)) + Model2._bgg;
	// _torque2 = Model2._bgg;
	// _torque2 = _torque;
	// cout<<"model1 A :\n"<<Model._A<<endl;
	// cout<<"model2 A :\n"<<Model2._A<<endl;
	// cout<<"model1 bg :"<<Model._bgg.transpose()<<endl;
	// cout<<"model2 bg :"<<Model2._bgg.transpose()<<endl;

	// cout<<"q_des :"<<_q_des2.transpose()<<endl;
	// cout<<"q_    :"<<_q2.transpose()<<endl;
	// cout<<"qd d  :"<<_qdot_des2.transpose()<<endl;
	// cout<<"qd    :"<<_qdot2.transpose()<<endl;
	// cout<<"2 q des :"<<_q_des2.transpose()<<endl;
	// cout<<"2 q now :"<<_q2.transpose()<<endl;
	// cout<<"2 qd des :"<<_qdot_des2.transpose()<<endl;
	// cout<<"2 qd now :"<<_qdot2.transpose()<<endl;
	// cout<<"2 torque :"<<_torque2.transpose()<<endl;
}

void CController::CLIK()
{
	_torque.setZero();

	_x_err_hand.segment(0, 3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3, 3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);
	// _J_bar_hands = CustomMath::DampedWeightedPseudoInverse(_J_hands,_I*0.01,true);

	_qdot_des = _J_bar_hands * (_xdot_des_hand + _x_kp * (_x_err_hand));
	_q_des = _q + _dt * _qdot_des;

	for (int i = 0; i < 7; i++)
	{
		_A_diagonal(i, i) = Model._A(i, i);
	}

	_torque = _A_diagonal * (_kpj * (_q_des - _q) + _kdj * (_qdot_des - _qdot)) + Model._bgg;

	// cout << _torque.transpose() << endl;
	// _torque =  Model._A*(_kp*(_q_des - _q) + _kd*(_qdot_des - _qdot)) + Model._bg;
}

void CController::Initialize()
{

	_control_mode = 1; // 1: joint space, 2: task space(CLIK)

	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_kpj = 400.0;
	_kdj = 40.0;

	_x_kp = 20.0;

	timestep_cnt = 200001;
	_cnt_plan = 0;
	_bool_plan.setZero(timestep_cnt + 1); // 20
	_time_plan.resize(timestep_cnt + 1);  // 20
	_time_plan.setConstant(1.0);

	_q_home.setZero(_k);
	_q_home(0) = 0.374;
	_q_home(1) = -1.02;
	_q_home(2) = 0.245;
	_q_home(3) = -1.51;
	_q_home(4) = 0.0102;
	_q_home(5) = 0.655;
	_q_home(6) = 0.3;

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_max_joint_position.setZero(7);
	_min_joint_position.setZero(7);

	_min_joint_position(0) = -2.9671;
	_min_joint_position(1) = -1.8326;
	_min_joint_position(2) = -2.9671;
	_min_joint_position(3) = -3.1416;
	_min_joint_position(4) = -2.9671;
	_min_joint_position(5) = -0.0873;
	_min_joint_position(6) = -2.9671;

	_max_joint_position(0) = 2.9671;
	_max_joint_position(1) = 1.8326;
	_max_joint_position(2) = 2.9671;
	_max_joint_position(3) = 0.0;
	_max_joint_position(4) = 2.9671;
	_max_joint_position(5) = 3.8223;
	_max_joint_position(6) = 2.9671;

	_max_joint_velocity.setZero(7);
	_min_joint_velocity.setZero(7);
	_min_joint_velocity(0) = -2.62;
	_min_joint_velocity(1) = -2.62;
	_min_joint_velocity(2) = -2.62;
	_min_joint_velocity(3) = -2.62;
	_min_joint_velocity(4) = -5.26;
	_min_joint_velocity(5) = -4.18;
	_min_joint_velocity(6) = -5.26;
	_max_joint_velocity(0) = 2.62;
	_max_joint_velocity(1) = 2.62;
	_max_joint_velocity(2) = 2.62;
	_max_joint_velocity(3) = 2.62;	
	_max_joint_velocity(4) = 5.26;
	_max_joint_velocity(5) = 4.18;
	_max_joint_velocity(6) = 5.26;

	check_save_log_once = true;
	save_q_once = true;

	check_7 = 0;
	a = 0;
	cnt2 = 0;
	_rat = 0;
	_ratt = 0;
	ratrat = 135;

	prevent_duplicate = 0.01;
	_tmp = 0.0;

	_I.setIdentity(7, 7);

	///////////////hand 1 ////////////////

	save_q_once1 = true;
	_q.setZero(_k);
	_qdot.setZero(_k);
	_torque.setZero(_k);

	_J_hands.setZero(6, _k);
	_J_bar_hands.setZero(_k, 6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	_q_des.setZero(_k);
	_qdot_des.setZero(_k);
	_q_goal.setZero(_k);
	_qdot_goal.setZero(_k);

	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	_pos_goal_hand.setZero(); // 3x1
	_rpy_goal_hand.setZero(); // 3x1

	JointTrajectory.set_size(_k);
	_A_diagonal.setZero(_k, _k);

	_x_err_hand.setZero(6);
	_R_des_hand.setZero();

	_q_order.setZero(7);
	_q_home_old.setZero(7);

	///// hand 2 ////////
	save_q_once2 = true;
	_q2.setZero(_k);
	_qdot2.setZero(_k);
	_torque2.setZero(_k);

	_J_hands2.setZero(6, _k);
	_J_bar_hands2.setZero(_k, 6);

	_x_hand2.setZero(6);
	_xdot_hand2.setZero(6);

	_q_des2.setZero(_k);
	_qdot_des2.setZero(_k);
	_q_goal2.setZero(_k);
	_qdot_goal2.setZero(_k);

	_x_des_hand2.setZero(6);
	_xdot_des_hand2.setZero(6);
	_x_goal_hand2.setZero(6);
	_xdot_goal_hand2.setZero(6);

	_pos_goal_hand2.setZero(); // 3x1
	_rpy_goal_hand2.setZero(); // 3x1

	JointTrajectory2.set_size(_k);
	_A_diagonal2.setZero(_k, _k);

	_x_err_hand2.setZero(6);
	_R_des_hand2.setZero();

	_q_order2.setZero(7);
	_q_home_old2.setZero(7);
	std::uniform_real_distribution<double> random_motion_time(0.0, 1.0);

	// Define rng
	std::mt19937 rng(std::random_device{}());
}

void CController::save_stack()
{
	if (check_save_log_once == true)
	{
		check_save_log_once = false;
		if (a != 0)
		{
			output.open("/home/kist-robot2/Desktop/KDH/franka_panda_3/save_data/dual_panda/panda_qqd_0423.txt", ios::app);
			for (int i = 0; i < a; i++)
			{
				check_7++;
				if (check_7 == 4 * _k + 1)
				{
					output << log_output[i] << endl;
					check_7 = 0;
				}
				else
				{
					output << log_output[i] << ",";
				}
			}
			output.close();
			a = 0;
			cout << "first log time" << endl;
		}
		else
		{
			cout << "first log time but a == 0" << endl;
			check_save_log_once = true;
		}
	}
	else
	{
		if (a != 0)
		{
			output.open("/home/kist-robot2/Desktop/KDH/franka_panda_3/save_data/dual_panda/panda_qqd_0423.txt", ios::app);
			for (int i = 0; i < a; i++)
			{
				check_7++;
				if (check_7 == 4 * _k + 1)
				{
					output << log_output[i] << endl;
					check_7 = 0;
				}
				else
				{
					output << log_output[i] << ",";
				}
			}
			output.close();
			a = 0;
			cout << "log" << endl;
		}
		else
		{
			cout << " a == 0 " << endl;
		}
	}
}

float CController::rand_FloatRange(float a, float b)
{
	if (_t < 0.01)
	{
		srand((unsigned)time(NULL));
	}

	// float k = (float)rand();
	// cout<<"k : "<<k<<endl;
	return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}

double CController::generateRandomReal(double a, double b)
{
	static std::random_device rd;  // Static to ensure initialization only once
	static std::mt19937 gen(rd()); // Static to ensure initialization only once

	// Define a distribution for real numbers
	std::uniform_real_distribution<double> dis(0.0, 1.0); // Random real numbers between 0.0 and 1.0

	// return dis(gen); // Generate and return a random real number

	return ((b - a) * (dis(gen) / 1.0)) + a;
	// return dis(gen);
}

// namespace py = pybind11;
// PYBIND11_MODULE(controller, m)
// {
//   m.doc() = "pybind11 for controller";

//   py::class_<CController>(m, "CController")
//       .def(py::init())
//       .def("read", &CController::read_pybind)
// 	  .def("control_mujoco", &CController::control_mujoco)
// 	  .def("write", &CController::write_pybind);

// #ifdef VERSION_INFO
//   m.attr("__version__") = VERSION_INFO;
// #else
//   m.attr("__version__") = "dev";
// #endif

// //   m.attr("TEST") = py::int_(int(42));
// }