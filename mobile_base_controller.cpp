/**
 * @file controller.cpp
 * @brief Controller file 
 */

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/robusser_robot.urdf";

#include "redis_keys.h"

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

Vector2d repulsive_torque(Vector2d q, Vector2d q_obs) {
	double eta = 5.0;
	double dist = (q - q_obs).norm();
	double d0 = 0.3;
	Vector2d torque = Vector2d::Zero();

	if (dist <= d0) {
		torque = eta*(1/dist - 1/d0) * (q - q_obs) / pow(dist, 3);
	}
	
	return torque;
}

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task->_use_interpolation_flag = true;
	posori_task->_otg->setMaxLinearVelocity(0.8);
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400;
	posori_task->_kv_pos = 40;
	posori_task->_kp_ori = 400;
	posori_task->_kv_ori = 40;

	// set the current EE posiiton as the desired EE position
	Vector3d x = Vector3d::Zero(3);
	robot->position(x, control_link, control_point);
	posori_task->_desired_position = x;

	// partial joint task to control the mobile base 
	vector<int> base_joint_selection{0, 1, 2};
	auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	base_task->_use_velocity_saturation_flag = true;
	base_task->_saturation_velocity << 0.2, 0.2, 0.3;  // adjust based on speed
	
	VectorXd base_task_torques = VectorXd::Zero(dof);
	base_task->_kp = 400;
	base_task->_kv = 40;
	
	VectorXd base_pose_desired = initial_q.head(3);
	base_pose_desired << -3.0, -1.0, 0.0;
	base_task->_desired_position = base_pose_desired;

	// joint (posture) task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto arm_joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	arm_joint_task->_use_interpolation_flag = false;

	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 20;

	// set the desired posture
	VectorXd q_desired = initial_q.tail(7);
	q_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired *= M_PI/180.0;
	arm_joint_task->_desired_position = q_desired;

	// gripper (posture) task
	vector<int> gripper_selection{10, 11};
	auto gripper_joint_task = new Sai2Primitives::PartialJointTask(robot, gripper_selection);
	gripper_joint_task->_use_interpolation_flag = false;

	VectorXd gripper_torques = VectorXd::Zero(dof);
	gripper_joint_task->_kp = 100;
	gripper_joint_task->_kv = 20;

	// set the desired posture
	VectorXd gripper_desired = VectorXd::Zero(2);
	gripper_desired << 0.02, -0.02;
	gripper_joint_task->_desired_position = gripper_desired;
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	// Vector2d q_obs = Vector2d(-2.66, -1.75);
	Vector2d q_obs;
	Vector2d vertex1 = Vector2d(-3.77, -2.6);
	Vector2d vertex2 = Vector2d(-3.77, -0.9);
	Vector2d vertex3 = Vector2d(-1.55, -0.9);
	Vector2d vertex4 = Vector2d(-1.55, -2.6);

	const int num = 10;
	VectorXd obs_x = VectorXd::LinSpaced(num, vertex2[0], vertex3[0]);
	VectorXd obs_y = VectorXd::LinSpaced(num, vertex1[1], vertex2[1]);

	Vector2d obstacles[4*num - 4];
	int counter = 0;

	for (int i = 0; i < num - 1; i++) {
		obstacles[counter] << vertex1[0], obs_y[i];
		counter++;
	}
	for (int i = 0; i < num - 1; i++) {
		obstacles[counter] << obs_x[i], vertex2[1];
		counter++;
	}
	for (int i = num - 1; i > 0; i--) {
		obstacles[counter] << vertex3[0], obs_y[i];
		counter++;
	}
	for (int i = num - 1; i > 0; i--) {
		obstacles[counter] << obs_x[i], vertex4[1];
		counter++;
	}

	bool goalCloseToObstacle = false;
	double distance = 10;
	double temp;
	bool repulseOn = true;

	for (int i = 0; i < 4*num - 4; i++) {
		temp = (base_pose_desired.head(2) - obstacles[i]).norm();
		if (temp < distance) {
			distance = temp;
		}
	}

	if (distance < 0.3) {
		goalCloseToObstacle = true;
	}

	//cout << distance << endl;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		
		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->position(x, control_link, control_point);

		Vector2d current_xy = robot->_q.head(2);
		robot->updateModel();

		// sample desired set points
		//cout << robot->_q << endl << endl;
		//cout << x.transpose() << endl << endl;

		// set controller inputs
		posori_task->_desired_position = x;
		base_task->_desired_position = base_pose_desired;
		arm_joint_task->_desired_position = q_desired;
		gripper_joint_task->_desired_position = gripper_desired;

		auto base_velocity = robot->_dq.head(2);

		//cout << (current_xy - base_pose_desired.head(2)).norm() << endl;

		// update task model and set hierarchy
		// /*
		// 	arm-driven motion hieararchy: arm -> base -> arm nullspace
		// */ 
		// N_prec.setIdentity();
		// posori_task->updateTaskModel(N_prec);
		// N_prec = posori_task->_N;	
		// base_task->updateTaskModel(N_prec);
		// N_prec = base_task->_N;	
		// arm_joint_task->updateTaskModel(N_prec);
		// gripper_joint_task->updateTaskModel(N_prec);

		/*
			base-driven motion hieararchy: base -> arm -> arm nullspace 
		*/
		N_prec.setIdentity();
		base_task->updateTaskModel(N_prec);
		N_prec = base_task->_N;		
		arm_joint_task->updateTaskModel(N_prec);
		gripper_joint_task->updateTaskModel(N_prec);
		//posori_task->updateTaskModel(N_prec);
		//N_prec = posori_task->_N;	

		// obstacle avoidance
		VectorXd base_task_torques_obs = VectorXd::Zero(dof);

		if (repulseOn){
			for (int i = 0; i < 4*num - 4; i++) {
				base_task_torques_obs.head(2) += repulsive_torque(current_xy, obstacles[i]);
			}
			cout << "REPULSE ON!" << endl;
		}

		if (time > 1.0 && base_velocity.norm() < 0.01 && goalCloseToObstacle && (current_xy - base_pose_desired.head(2)).norm() > 0.1 && repulseOn) {
			cout << "SWITCHING OFF REPULSE";
			repulseOn = false;
		}

		// compute torques
		//posori_task->computeTorques(posori_task_torques);
		base_task->computeTorques(base_task_torques);
		arm_joint_task->computeTorques(arm_joint_task_torques);
		gripper_joint_task->computeTorques(gripper_torques);
		//cout << base_task_torques.head(2).transpose() << endl;
		//cout << base_task_torques.transpose() << endl;

		command_torques = base_task_torques + base_task_torques_obs + arm_joint_task_torques + gripper_torques;

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}
