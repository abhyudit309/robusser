/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::cs225a::robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::robot::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::robot::actuators::fgc";

const std::string DISHWASHER_JOINT_ANGLES_KEY = "sai2::cs225a::dishwasher::sensors::q";
const std::string DISHWASHER_JOINT_VELOCITIES_KEY = "sai2::cs225a::dishwasher::sensors::dq";
const std::string DISHWASHER_JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::dishwasher::actuators::fgc";

const std::string X_VEL_KEY = "sai2::cs225a::project::control::xv";
const std::string Y_VEL_KEY = "sai2::cs225a::project::control::yv";

const std::string SIMULATION_LOOP_DONE_KEY = "cs225a::simulation::done";
const std::string CONTROLLER_LOOP_DONE_KEY = "cs225a::controller::done";
const std::string EE_X_POS_KEY = "sai2::cs225a::project::control::xp";
const std::string EE_Y_POS_KEY = "sai2::cs225a::project::control::yp";
const std::string EE_Z_POS_KEY = "sai2::cs225a::project::control::zp";
const std::string ORI_X_POS_KEY = "sai2::cs225a::project::control::xo";
const std::string ORI_Y_POS_KEY = "sai2::cs225a::project::control::yo";
const std::string ORI_Z_POS_KEY = "sai2::cs225a::project::control::zo";
const std::string GRIPPER_POS_KEY = "sai2::cs225a::project::control::gp";
