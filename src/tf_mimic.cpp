/*
 * tf_publisher.cpp
 *
 *  Created on: December 18, 2016
 *      Author: Silvia Cruciani
*/

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char *argv[]){
	//this node publishes the transformation read from a file in tf
	ros::init(argc, argv, "tf_publisher");	
	ros::NodeHandle n("~");

	std::string base_frame, end_frame, detected_frame;
	bool invert_transform;
	//read names of base frame and end frame from parameters
	n.param<std::string>("base_frame", base_frame, "world_tag");
	n.param<std::string>("end_frame", end_frame, "kinect2_link");
	n.param<std::string>("detected_frame", detected_frame, "world_tag2");
	n.param<bool>("inverted_transform", invert_transform, false);

	if(invert_transform){
		ROS_INFO("publishing transform between %s and %s", end_frame.c_str(), base_frame.c_str());
	}
	else{
		ROS_INFO("publishing transform between %s and %s", base_frame.c_str(), end_frame.c_str());
	}

	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	ros::Rate r(100.0);

	//read the transformation 
	tf::StampedTransform transform;
	bool observed_tf = false;
	while (ros::ok() and !observed_tf) {
		try{
			ROS_INFO_STREAM("Waiting for transform from " << end_frame << " to " << detected_frame);
			if(invert_transform){
				listener.waitForTransform(end_frame, detected_frame, ros::Time::now(), ros::Duration(10.0));
				listener.lookupTransform(end_frame, detected_frame, ros::Time(0), transform);
			}
			else{
				listener.waitForTransform(detected_frame, end_frame, ros::Time::now(), ros::Duration(10.0));
				listener.lookupTransform(detected_frame, end_frame, ros::Time(0), transform);
			}
			std::cout<<"---- READ: "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;
			std::cout<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<std::endl;
			observed_tf = true;
		}
		catch (tf::TransformException ex){
			ROS_ERROR("Failed to retrieve tf to mimic. Error: %s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}
	

	tf::StampedTransform transform_bk=transform;
	while(ros::ok() and observed_tf){
		std::cout << observed_tf << std::endl;
	    try{
	    	if(invert_transform){
	    		listener.waitForTransform(end_frame, detected_frame, ros::Time::now(), ros::Duration(3.0));
				listener.lookupTransform(end_frame, detected_frame, ros::Time(0), transform);
	    	}
	    	else{
	    		listener.waitForTransform(detected_frame, end_frame, ros::Time::now(), ros::Duration(3.0));
				listener.lookupTransform(detected_frame, end_frame, ros::Time(0), transform);
			}
			std::cout<<"---- READ: "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;
	    	std::cout<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<std::endl;
	    }
	    catch (tf::TransformException ex){
	    	ROS_ERROR("%s",ex.what());
	    	ros::Duration(1.0).sleep();
	    	transform = transform_bk;
	    }
	    transform_bk = transform;
	    std::cout<<"---- SENDING: "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;
	    std::cout<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<std::endl;
		if(invert_transform){
	    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), end_frame, base_frame));
	    }
	    else{
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame, end_frame));
		}
		r.sleep();
	}

	return 0;
}
