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
