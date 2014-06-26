/*
 * ProcessManager.cpp
 *
 *  Created on: May 22, 2014
 *      Author: bcalli
 */

#include "dr_util/process_manager.h"

dr::ProcessManager::ProcessManager() {
	n_ = NULL;
	initialized_ = false;
}

dr::ProcessManager::ProcessManager(ros::NodeHandle * n) {
	initialize(n);
}

void dr::ProcessManager::initialize(ros::NodeHandle * n){
	n_ = n;
	getGeneralParameters();
	getProcessSpecificParameters();
	reset();
	initializeBasicMessagesServices();
	initializeProcessSpecificMessagesServices();
	initialized_ = true;
}

void dr::ProcessManager::reset(){
	trigger_start_received_ = false;
	trigger_stop_received_ = false;
	trigger_abort_received_ = false;
}

void dr::ProcessManager::getGeneralParameters(){

	if (!n_->getParam("wait_external_trigger_for_start", wait_external_trigger_for_start_)) {
		bool default_val = false;
		ROS_WARN("[process_manager]: Failed to get the time_wait_before_start parameter from the parameter server. Using the default value: %d", default_val);
		wait_external_trigger_for_start_ = default_val;
	}

	if (!n_->getParam("time_wait_before_start", time_wait_before_start_)) {
		int default_val = 0;
		ROS_WARN("[process_manager]: Failed to get the time_wait_before_start parameter from the parameter server. Using the default value: %d", default_val);
		time_wait_before_start_ = default_val;
	}

}

void dr::ProcessManager::initializeBasicMessagesServices(){
	srv_clnt_start_ = n_->serviceClient<std_srvs::Empty>("start_process");
	srv_clnt_stop_ = n_->serviceClient<std_srvs::Empty>("stop_process");
	srvsrv_ext_start_trig_ = n_->advertiseService("external_start_trigger", &ProcessManager::callbackExtStartTrigger, this);
	srvsrv_ext_stop_trig_ = n_->advertiseService("external_stop_trigger", &ProcessManager::callbackExtStopTrigger, this);
	srvsrv_ext_abort_trig_ = n_->advertiseService("external_abort_trigger", &ProcessManager::callbackExtAbortTrigger, this);
}

bool dr::ProcessManager::callbackExtStartTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	trigger_start_received_ = true;
	return true;
}

bool dr::ProcessManager::callbackExtStopTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	trigger_stop_received_ = true;
	return true;
}

bool dr::ProcessManager::callbackExtAbortTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	trigger_abort_received_ = true;
	return true;
}

void dr::ProcessManager::sendStartCommand(){
	if(!initialized_){
		ROS_INFO("[process_manager]: Process manager is used before being initialized. Cannot execute start process command.");
		return;
	}
	std_srvs::Empty srv;
	srv_clnt_start_.call(srv);
}

void dr::ProcessManager::sendStopCommand(){
	if(!initialized_){
		ROS_INFO("[process_manager]: Process manager is used before being initialized. Cannot execute stop process command.");
		return;
	}
	std_srvs::Empty srv;
	srv_clnt_stop_.call(srv);
}

void dr::ProcessManager::spin(){
	if(!initialized_){
		ROS_INFO("[process_manager]: Process manager is used before being initialized. Cannot execute spin command.");
		return;
	}

	sleep(time_wait_before_start_);

	ros::Rate loop_rate(10);


	prepareSystem();
	ros::spinOnce();

	while(ros::ok() && !trigger_abort_received_){
		reset();

		while(ros::ok() && wait_external_trigger_for_start_ && !trigger_start_received_ && !trigger_abort_received_){
			loop_rate.sleep();
			ros::spinOnce();
		}

		sendStartCommand();

		while(ros::ok() && !trigger_stop_received_ && !trigger_abort_received_){

			step();
			loop_rate.sleep();
			ros::spinOnce();
		}

		sendStopCommand();
		resetSystem();
	}

}

dr::ProcessManager::~ProcessManager() {

}

