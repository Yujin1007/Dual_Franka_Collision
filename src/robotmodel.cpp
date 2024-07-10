#include "robotmodel.h"
#define JDOF 7

CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

    _k = JDOF;
    _id_hand = 7;

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);

    _q.setZero(_k);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _A.setZero(_k,_k);
    _g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);
	_bgg.setZero(_k);

    _J_hand.setZero(6,_k);
    _J_tmp.setZero(6,_k);

    _position_local_task_hand.setZero(); // 3x1
	_tmp_position_local_task_hand.setZero(); // 3x1
	// _tmp_position_local_task_hand(0) = 0.088;

    _x_hand.setZero(); // 3x1
    _R_hand.setZero(); // 3x3

    _xdot_hand.setZero(6);

	set_robot_config();
    load_model();
}


void CModel::load_model()
{   
    // RigidBodyDynamics::Addons::URDFReadFromFile("../model/panda_arm_hand.urdf", &_model, false, true);
	// RigidBodyDynamics::Addons::URDFReadFromFile("../model/fr3.urdf", &_model, false, false);
	RigidBodyDynamics::Addons::URDFReadFromFile("/home/kist-robot2/Franka/franka_panda_xl_steel/fr3_xl_huscky/fr3.urdf", &_model, false, false);
    cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

    _bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;

}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); // update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; // check kinematics update
	
}

void CModel::update_dynamics()
{
			
	if (_bool_kinematics_update == true)
	{	
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false); // update inertia matrix
		// cout<<"_A\n"<<_A<<endl;
		RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _g, NULL); // get _g
		// cout<<"_g\n"<<_g<<endl;

		RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bgg, NULL); // get _g+_b
		_b = _bgg - _g; //get _b
		// _b = _bg - _g; //get _b
		// cout<<"_bgg"<<_bgg.transpose()<<endl;
		
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; // check kinematics update
	// cout<<"dynamics calculation done \n\n"<<endl;
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{
		_J_hand.setZero();
		_J_tmp.setZero();	

		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_hand, _tmp_position_local_task_hand, _J_tmp, false); // update kinematc : false
		_J_hand.block<3, 7>(0, 0) = _J_tmp.block<3, 7>(3, 0); // linear : last three entries -> first three entries
		_J_hand.block<3, 7>(3, 0) = _J_tmp.block<3, 7>(0, 0); // angular : first three entries -> last three entries

		_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}

}

void CModel::calculate_EE_positions_orientations()
{
    if (_bool_kinematics_update == true)
	{
		_x_hand.setZero();
		_R_hand.setZero();

		_x_hand = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_hand, _position_local_task_hand, false);
		_R_hand = RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_hand, false).transpose();

	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}

void CModel::set_robot_config(){
	
	_min_joint_position(0) = -2.7437;
	_min_joint_position(1) = -1.7837;
	_min_joint_position(2) = -2.9007;
	_min_joint_position(3) = -3.0421;
	_min_joint_position(4) = -2.8065;
	_min_joint_position(5) =  0.5445;
	_min_joint_position(6) = -3.0159;

	_max_joint_position(0) = 2.7437;
	_max_joint_position(1) = 1.7837;
	_max_joint_position(2) = 2.9007;
	_max_joint_position(3) = -0.1518;
	_max_joint_position(4) = 2.8065;
	_max_joint_position(5) = 4.5169;
	_max_joint_position(6) = 3.0159;

	_max_joint_velocity(0) = 2.62;
	_max_joint_velocity(1) = 2.62;
	_max_joint_velocity(2) = 2.62;
	_max_joint_velocity(3) = 2.62;
	_max_joint_velocity(4) = 5.26;
	_max_joint_velocity(5) = 4.18;
	_max_joint_velocity(6) = 5.26;
	_min_joint_velocity = -_max_joint_velocity;

}
