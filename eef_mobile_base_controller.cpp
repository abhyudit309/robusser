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

#define BASE_CONTROLLER 0
#define ARM_CONTROLLER  1

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
	bool autonomous = false;

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
	
	// Position and orientation task control
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 400;
	posori_task->_kv_pos = 40;
	posori_task->_kp_ori = 400;
	posori_task->_kv_ori = 40;

	// set the current EE posiiton as the desired EE position
	// Current EE X position
	Vector3d x = Vector3d::Zero(3);

	// Desired EE X position
	Vector3d xd = Vector3d::Zero(3);

	// circular trajectory
	double Amp = 0.1;
	double w = M_PI;
	robot->position(x, control_link, control_point);

	// Set the EE position (WHY DO WE DO THIS?)
	posori_task->_desired_position = x;


	// LIST OF GOAL EE POSES
	Vector3d eef_pose_array[3];
	eef_pose_array[0] << 0.3, 0.1, 0.5;
	eef_pose_array[1] << 0.1, 0.4, 0.6;
	eef_pose_array[2] << 0.5, 0.2, 0.4;


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
	Vector3d base_xyz;
	Vector2d base_velocity;
	base_xyz << initial_q(0), initial_q(1), 0.0;
	base_task->_desired_position = base_pose_desired;

	// array of base poses
	Vector3d base_pose_array[3];
	base_pose_array[0] << -1.0, -1.0, 0.0;
	base_pose_array[1] << -4.5, -3.0, 0.0;
	base_pose_array[2] << -3.0, -1.0, 0.0;

	// joint (posture) task
	vector<int> arm_joint_selection{3, 4, 5, 6, 7, 8, 9};
	auto arm_joint_task = new Sai2Primitives::PartialJointTask(robot, arm_joint_selection);
	arm_joint_task->_use_interpolation_flag = false;

	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 20;

	// set the desired posture
	VectorXd q_desired = initial_q.segment(3, 7);
	// q_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	// q_desired *= M_PI/180.0;
	arm_joint_task->_desired_position = q_desired;

	// gripper (posture) task
	vector<int> gripper_selection{10, 11};
	auto gripper_joint_task = new Sai2Primitives::PartialJointTask(robot, gripper_selection);
	gripper_joint_task->_use_interpolation_flag = false;

	VectorXd gripper_torques = VectorXd::Zero(dof);
	gripper_joint_task->_kp = 100;
	gripper_joint_task->_kv = 20;

	// set the desired posture
	VectorXd gripper_desired = initial_q.tail(2);
	// gripper_desired << 0.02, -0.02;
	gripper_joint_task->_desired_position = gripper_desired;
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	Vector2d q_obs;
	Vector2d vertex1 = Vector2d(-3.77, -2.6);
	Vector2d vertex2 = Vector2d(-3.77, -0.9);
	Vector2d vertex3 = Vector2d(-1.55, -0.9);
	Vector2d vertex4 = Vector2d(-1.55, -2.6);

	const int num = 10;
	VectorXd obs_x = VectorXd::LinSpaced(num, vertex2[0], vertex3[0]);
	VectorXd obs_y = VectorXd::LinSpaced(num, vertex1[1], vertex2[1]);

	Vector2d obstacles[4*num - 4];
	int j = 0;

	for (int i = 0; i < num - 1; i++) {
		obstacles[j] << vertex1[0], obs_y[i];
		j++;
	}
	for (int i = 0; i < num - 1; i++) {
		obstacles[j] << obs_x[i], vertex2[1];
		j++;
	}
	for (int i = num - 1; i > 0; i--) {
		obstacles[j] << vertex3[0], obs_y[i];
		j++;
	}
	for (int i = num - 1; i > 0; i--) {
		obstacles[j] << obs_x[i], vertex4[1];
		j++;
	}

	bool goalCloseToObstacle = false;
	double distance = 10;
	double temp;
	bool repulseOn = true;
	double switch_time = 0;

	int state = ARM_CONTROLLER;
	int counter = 0;

	if (autonomous) {
		base_pose_desired = base_pose_array[counter];
	}

	for (int i = 0; i < 4*num - 4; i++) {
		temp = (base_pose_desired.head(2) - obstacles[i]).norm();
		if (temp < distance) {
			distance = temp;
		}
	}

	if (distance < 0.3) {
		goalCloseToObstacle = true;
	}

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		
		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		base_xyz << robot->_q(0), robot->_q(1), 0.0;

		base_velocity = robot->_dq.head(2);

		robot->position(x, control_link, control_point);

		if (autonomous) {
			xd = eef_pose_array[counter] + base_xyz;

			if ((robot->_q.head(3) - base_pose_desired).norm() < 0.001 && state == BASE_CONTROLLER) {
				state = ARM_CONTROLLER;
				cout << robot->_q.head(3).transpose() << " reached!! BASE" << endl;
			} 

			if ((x - xd).norm() < 0.001 && state == ARM_CONTROLLER) {
				state = BASE_CONTROLLER;
				counter++;
				base_pose_desired = base_pose_array[counter];
				q_desired = robot->_q.segment(3, 7);
				cout << (x-base_xyz).transpose() << " reached!! ARM" << endl;
				// reseting values

				goalCloseToObstacle = false;
				distance = 10;
				repulseOn = true;
				switch_time = time;

				for (int i = 0; i < 4*num - 4; i++) {
					temp = (base_pose_desired.head(2) - obstacles[i]).norm();
					if (temp < distance) {
						distance = temp;
					}
				}

				if (distance < 0.3) {
					goalCloseToObstacle = true;
				}
			}
		}

		robot->updateModel();

		// set controller inputs
		posori_task->_desired_position = xd;
		base_task->_desired_position = base_pose_desired;
		arm_joint_task->_desired_position = q_desired;
		gripper_joint_task->_desired_position = gripper_desired;

		if (state == BASE_CONTROLLER) {
			// put base control code here
			//std::cout << "BASE" << std::endl;
			N_prec.setIdentity();
			base_task->updateTaskModel(N_prec);
			N_prec = base_task->_N;		
			arm_joint_task->updateTaskModel(N_prec);
			gripper_joint_task->updateTaskModel(N_prec);

			// obstacle avoidance
			VectorXd base_task_torques_obs = VectorXd::Zero(dof);

			if (repulseOn){
				for (int i = 0; i < 4*num - 4; i++) {
					base_task_torques_obs.head(2) += repulsive_torque(base_xyz.head(2), obstacles[i]);
				}
			}

			if ((time - switch_time) > 1.0 && base_velocity.norm() < 0.01 && goalCloseToObstacle && (base_xyz.head(2) - base_pose_desired.head(2)).norm() > 0.1 && repulseOn) {
				cout << "SWITCHING OFF REPULSE";
				repulseOn = false;
			}

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			base_task->computeTorques(base_task_torques);
			arm_joint_task->computeTorques(arm_joint_task_torques);
			gripper_joint_task->computeTorques(gripper_torques);

			command_torques = base_task_torques + base_task_torques_obs + arm_joint_task_torques + gripper_torques;

		}

		else if (state == ARM_CONTROLLER) {
			// put arm control code here
			//std::cout << "ARM" << std::endl;
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;	
			base_task->updateTaskModel(N_prec);
			N_prec = base_task->_N;	
			arm_joint_task->updateTaskModel(N_prec);
			gripper_joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			base_task->computeTorques(base_task_torques);
			arm_joint_task->computeTorques(arm_joint_task_torques);
			gripper_joint_task->computeTorques(gripper_torques);

			command_torques = posori_task_torques + base_task_torques + arm_joint_task_torques + gripper_torques;
		}

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

		if (counter >= 3 && autonomous) {
			cout << "EXIT!";
			break;
		}
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}
