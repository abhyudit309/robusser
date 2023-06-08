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

#include <signal.h>
bool runloop = true;
void sighandler(int sig){ runloop = false; }
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

using namespace std;
using namespace Eigen;

//const string robot_file = "./resources/robusser_robot.urdf";
const string dishwasher_file = "./resources/dishwasher_robot.urdf";

#include "redis_keys.h"

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto dishwasher = new Sai2Model::Sai2Model(dishwasher_file, false);
	dishwasher->_q = redis_client.getEigenMatrixJSON(DISHWASHER_JOINT_ANGLES_KEY);
	VectorXd initial_q = dishwasher->_q;
	dishwasher->updateModel();

	// prepare controller
	int dof = dishwasher->dof();
	VectorXd dishwasher_command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint (posture) task
	auto arm_joint_task = new Sai2Primitives::JointTask(dishwasher);
	arm_joint_task->_use_interpolation_flag = false;
    arm_joint_task->_use_velocity_saturation_flag = false;
    
	VectorXd arm_joint_task_torques = VectorXd::Zero(dof);
	arm_joint_task->_kp = 100;
	arm_joint_task->_kv = 20;

	// set the desired posture
	VectorXd q_desired = initial_q;
	//q_desired.setZero();
    //q_desired << -0.5, 1.6;
	arm_joint_task->_desired_position = q_desired;

    redis_client.createReadCallback(0);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;


	while (runloop) {
		// wait for next scheduled loop
		// timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

        // read simulation state
        fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

        // run controller loop when simulation loop is done
        if (fSimulationLoopDone) {

            redis_client.executeReadCallback(0);
            
            // read robot state from redis
            dishwasher->_q = redis_client.getEigenMatrixJSON(DISHWASHER_JOINT_ANGLES_KEY);
            dishwasher->_dq = redis_client.getEigenMatrixJSON(DISHWASHER_JOINT_VELOCITIES_KEY);
            dishwasher->updateModel();


            // set controller inputs
            if (controller_counter % 5000 == 0 || controller_counter % 5000 == 3000) { 
                q_desired(1) = 1.8 - q_desired(1);
            }
            if (controller_counter % 5000 == 300 || controller_counter % 5000 == 2700) { 
                q_desired(0) *= -1;
            }
            arm_joint_task->_desired_position = q_desired;

            N_prec.setIdentity();
            arm_joint_task->updateTaskModel(N_prec);

            // compute torques
            arm_joint_task->computeTorques(arm_joint_task_torques);

            dishwasher_command_torques = arm_joint_task_torques;

            // send to redis
            redis_client.setEigenMatrixJSON(DISHWASHER_JOINT_TORQUES_COMMANDED_KEY, dishwasher_command_torques);

            // ask for next simulation loop
            fSimulationLoopDone = false;
            redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));

            controller_counter++;
        }

        // controller loop is done
        fControllerLoopDone = true;
        redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
	}

    // controller loop is turned off
    fControllerLoopDone = false;
    redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(DISHWASHER_JOINT_TORQUES_COMMANDED_KEY, 0 * dishwasher_command_torques);

	return 0;
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
    assert(x == "false" || x == "true");
    return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
    return b ? "true" : "false";
}
