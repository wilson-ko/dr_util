/*
 * ProcessManager.h
 *
 *  Created on: May 22, 2014
 *      Author: bcalli
 */

#ifndef PROCESSMANAGER_H_
#define PROCESSMANAGER_H_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
namespace dr{
	class ProcessManager {


		bool initialized_,trigger_start_received_,trigger_abort_received_;
		int time_wait_before_start_;
		bool wait_external_trigger_for_start_;
		ros::ServiceClient srv_clnt_start_,srv_clnt_stop_;
		ros::ServiceServer srvsrv_ext_start_trig_,srvsrv_ext_stop_trig_, srvsrv_ext_abort_trig_;

		void reset();
		void sendStartCommand();
		void sendStopCommand();
		void getProcessParameters();
		void initMessagesServices();
		void initializeBasicMessagesServices();
		void getGeneralParameters();

		bool callbackExtStartTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool callbackExtStopTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool callbackExtAbortTrigger(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	protected:
		ros::NodeHandle * n_;
		bool trigger_stop_received_;

		ProcessManager();
		ProcessManager(ros::NodeHandle * n);

		virtual void prepareSystem() = 0;
		virtual void resetSystem() = 0;
		virtual void getProcessSpecificParameters() = 0;
		virtual void initializeProcessSpecificMessagesServices() = 0;
		virtual void step() = 0;

		virtual ~ProcessManager();
	public:

		void initialize(ros::NodeHandle * n);
		void spin();


	};
}

#endif /* PROCESSMANAGER_H_ */
