/**
 * @file simviz.cpp
 * @brief Simulation and visualization for mobile base with panda robot
 * 
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "parser/UrdfToSai2Graphics.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <typeinfo>


#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}
bool fSimulationLoopDone = false;
bool fControllerLoopDone = true; // initialize as true for first loop

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_robusser.urdf";
const string robot_file = "./resources/robusser_robot.urdf";
const string dishwasher_file = "./resources/dishwasher_robot.urdf";
const string robot_name = "robusser";
const string dishwasher_name = "dishwasher";
const string camera_name = "camera_fixed";

const vector<string> object_names = {"plate1", "plate2", "glass1", "glass2" };
vector<Vector3d> object_pos = {Vector3d(0, 0, -0.21), Vector3d(0, 0.65, -0.21),
							   Vector3d(0.4, 0, -0.21), Vector3d(0.4, 0.3, -0.21)};

vector<Vector3d> object_lin_vel;
vector<Quaterniond> object_ori;
vector<Vector3d> object_ang_vel;
const int n_objects = object_names.size();

// redis keys:
#include "redis_keys.h"

RedisClient redis_client;

// simulation function prototype
//void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* dishwasher, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* dishwasher, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

bool transXp = false;
bool transXn = false;
bool transYp = false;
bool transYn = false;

double x_vel = 0;
double y_vel = 0;

bool transXpEE = false;
bool transXnEE = false;
bool transYpEE = false;
bool transYnEE = false;
bool transZpEE = false;
bool transZnEE = false;
bool transGpEE = false;
bool transGnEE = false;
bool transXpEEOri = false;
bool transXnEEOri = false;
bool transYpEEOri = false;
bool transYnEEOri = false;
bool transZpEEOri = false;
bool transZnEEOri = false;
double control_grip_pos = 0;
double control_x_pos = 0;
double control_y_pos = 0;
double control_z_pos = 0;
double control_x_ori = 0;
double control_y_ori = 0;
double control_z_ori = 0;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(207.0/255, 245.0/255, 252.0/255);  // set blue background 

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();
	auto dishwasher = new Sai2Model::Sai2Model(dishwasher_file, false);
	dishwasher->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);

	sim->setCoeffFrictionStatic(1);


	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

	sim->getJointPositions(dishwasher_name, dishwasher->_q);
	sim->getJointVelocities(dishwasher_name, dishwasher->_dq);
	dishwasher->updateKinematics();

	for (int i = 0; i < n_objects; ++i) {
		Vector3d _object_pos, _object_lin_vel, _object_ang_vel;
		Quaterniond _object_ori;
		sim->getObjectPosition(object_names[i], _object_pos, _object_ori);
		sim->setObjectPosition(object_names[i], object_pos[i], _object_ori);
		
		sim->getObjectVelocity(object_names[i], _object_lin_vel, _object_ang_vel);
		//object_pos.push_back(_object_pos);
		object_lin_vel.push_back(_object_lin_vel);
		object_ori.push_back(_object_ori);
		object_ang_vel.push_back(_object_ang_vel);
	}


	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenW;
	int windowH = 0.8 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "MMP Panda Example", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	glewInitialize();

	fSimulationRunning = true;
	auto camera = graphics->getCamera(camera_name);
	//camera->setStereomode();
	thread sim_thread(simulation, robot, dishwasher, sim, ui_force_widget);
	
	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		// graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(dishwasher_name, dishwasher);
		for (int i = 0; i < n_objects; ++i) {
			graphics->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		}
		//graphics->render(camera_name, width, height);
		camera->renderView(width, height);
		

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		if (transXp) {
			x_vel = 0.0005;
		}
		if (transXn) {
			x_vel = -0.0005;
		}
		if (!transXp && !transXn) {
			x_vel = 0;
		}
		if (transYp) {
			y_vel = 0.0005;
		}
		if (transYn) {
			y_vel = -0.0005;
		}
		if (!transYp && !transYn) {
			y_vel = 0;
		}
		if (transXpEE) {
			control_x_pos = 1;
		}
		if (transXnEE) {
			control_x_pos = -1;
		}
		if (!transXpEE && !transXnEE) {
			control_x_pos = 0;
		}
		if (transYpEE) {
			control_y_pos = 1;
		}
		if (transYnEE) {
			control_y_pos = -1;
		}
		if (!transYpEE && !transYnEE) {
			control_y_pos = 0;
		}
		if (transZpEE) {
			control_z_pos = 1;
		}
		if (transZnEE) {
			control_z_pos = -1;
		}
		if (!transZpEE && !transZnEE) {
			control_z_pos = 0;
		}
		if (transXpEEOri) {
			control_x_ori = 0.01;
		}
		if (transXnEEOri) {
			control_x_ori = -0.01;
		}
		if (!transXpEEOri && !transXnEEOri) {
			control_x_ori = 0;
		}
		if (transYpEEOri) {
			control_y_ori = 0.01;
		}
		if (transYnEEOri) {
			control_y_ori = -0.01;
		}
		if (!transYpEEOri && !transYnEEOri) {
			control_y_ori = 0;
		}
		if (transZpEEOri) {
			control_z_ori = 0.01;
		}
		if (transZnEEOri) {
			control_z_ori = -0.01;
		}
		if (!transZpEEOri && !transZnEEOri) {
			control_z_ori = 0;
		}
		if (transGpEE) {
			control_grip_pos = 0.05;
		}
		if (transGnEE) {
			control_grip_pos = -0.05;
		}
		if (!transGpEE && !transGnEE) {
			control_grip_pos = 0;
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* dishwasher, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd dishwasher_command_torques = VectorXd::Zero(dishwasher->dof());
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.setEigenMatrixJSON(DISHWASHER_JOINT_TORQUES_COMMANDED_KEY, dishwasher_command_torques);

	redis_client.createWriteCallback(0);
	redis_client.addDoubleToWriteCallback(0, X_VEL_KEY, x_vel);
	redis_client.addDoubleToWriteCallback(0, Y_VEL_KEY, y_vel);
	redis_client.addDoubleToWriteCallback(0, EE_X_POS_KEY, control_x_pos);
	redis_client.addDoubleToWriteCallback(0, EE_Y_POS_KEY, control_y_pos);
	redis_client.addDoubleToWriteCallback(0, EE_Z_POS_KEY, control_z_pos);
	redis_client.addDoubleToWriteCallback(0, ORI_X_POS_KEY, control_x_ori);
	redis_client.addDoubleToWriteCallback(0, ORI_Y_POS_KEY, control_y_ori);
	redis_client.addDoubleToWriteCallback(0, ORI_Z_POS_KEY, control_z_ori);
	redis_client.addDoubleToWriteCallback(0, GRIPPER_POS_KEY, control_grip_pos);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// init variables
	VectorXd g(dof);
	VectorXd dishwasher_g(dishwasher->dof());

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	while (fSimulationRunning) {
		// fTimerDidSleep = timer.waitForNextLoop();

		if(fControllerLoopDone || fRobotLinkSelect) {
			// get gravity torques
			robot->gravityVector(g);
			dishwasher->gravityVector(dishwasher_g);

			// read arm torques from redis and apply to simulated robot
			if(fControllerLoopDone) {
				command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
				dishwasher_command_torques = redis_client.getEigenMatrixJSON(DISHWASHER_JOINT_TORQUES_COMMANDED_KEY);
			}
			else {
				command_torques.setZero();
				dishwasher_command_torques.setZero();
			}

			ui_force_widget->getUIForce(ui_force);
			ui_force_widget->getUIJointTorques(ui_force_command_torques);

			// if (fRobotLinkSelect)
			// 	sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + g);
			// else
			// 	sim->setJointTorques(robot_name, command_torques + g);
			sim->setJointTorques(dishwasher_name, dishwasher_command_torques + dishwasher_g);
			// integrate forward
			double curr_time = timer.elapsedTime();
			double loop_dt = curr_time - last_time; 
			// sim->integrate(loop_dt);
			sim->integrate(0.001);

			// read joint positions, velocities, update model
			// sim->getJointPositions(robot_name, robot->_q);
			// sim->getJointVelocities(robot_name, robot->_dq);
			// robot->updateModel();

			sim->getJointPositions(dishwasher_name, dishwasher->_q);
			sim->getJointVelocities(dishwasher_name, dishwasher->_dq);
			dishwasher->updateModel();
			cout << "Position: " << dishwasher->_q << endl;
			cout << "Velocity: " << dishwasher->_dq << endl;

			// write new robot state to redis
			redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
			redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

			redis_client.setEigenMatrixJSON(DISHWASHER_JOINT_ANGLES_KEY, dishwasher->_q);
			redis_client.setEigenMatrixJSON(DISHWASHER_JOINT_VELOCITIES_KEY, dishwasher->_dq);

			for (int i = 0; i < n_objects; ++i) {
				sim->getObjectPosition(object_names[i], object_pos[i], object_ori[i]);
				sim->getObjectVelocity(object_names[i], object_lin_vel[i], object_ang_vel[i]);
			}

			// simulation loop is done
            fSimulationLoopDone = true;

            // ask for next control loop
            fControllerLoopDone = false;

			redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
            redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone)); // ask for next control loop

			redis_client.executeWriteCallback(0);

			//update last time
			last_time = curr_time;
		}

        // read controller state
        fControllerLoopDone = string_to_bool(redis_client.get(CONTROLLER_LOOP_DONE_KEY));
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		case GLFW_KEY_R:
			transXpEE = set;
			break;
		case GLFW_KEY_T:
			transXnEE = set;
			break;
		case GLFW_KEY_F:
			transYpEE = set;
			break;
		case GLFW_KEY_G:
			transYnEE = set;
			break;
		case GLFW_KEY_V:
			transZpEE = set;
			break;
		case GLFW_KEY_B:
			transZnEE = set;
			break;
		case GLFW_KEY_Y:
			transXpEEOri = set;
			break;
		case GLFW_KEY_U:
			transXnEEOri = set;
			break;
		case GLFW_KEY_H:
			transYpEEOri = set;
			break;
		case GLFW_KEY_J:
			transYnEEOri = set;
			break;
		case GLFW_KEY_N:
			transZpEEOri = set;
			break;
		case GLFW_KEY_M:
			transZnEEOri = set;
			break;
		case GLFW_KEY_X:
			transGpEE = set;
			break;
		case GLFW_KEY_C:
			transGnEE = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
