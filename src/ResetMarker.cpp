/*********************************
 * ResetMarker.h
 * Marker for reseting robot position and simulation position
 * Based on OBO Worker Navigation system
 * Author : Theppasith N. <theppasith.n@hiveground.com>
 * 13 - Mar - 2018
 *********************************/
#include <ros/ros.h>
#include <string>
#include <iostream>
#include "obo_rviz_util/ResetMarker.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
namespace rviz
{
    // Constructor
    ResetMarker::ResetMarker()
    {
        robot_name_property_ = new StringProperty("Gazebo Model", "robot",
                                                  "The string indicates Gazebo Robot Parent model name",
                                                   getPropertyContainer(), SLOT( updateTopic() ), this );
        robot_model_name_ = "robot";
        ROS_INFO("[ResetMarker] Initialize Completed !");
    }
    ResetMarker::~ResetMarker()
    {
        ROS_INFO("[ResetMarker] Quitting");
    }

    // Abstract function from Rviz
    void ResetMarker::onInitialize()
    {
        // Super init
        PoseTool::onInitialize();
        // Button name set
        setName("Reset Pose");
        // Assignment Service client
        stage_ros_client_ = nh_.serviceClient<stage_ros::SetPose>("/stage_ros/set_pose");
        gazebo_ros_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        // executor_client_ = nh_.serviceClient<std_srvs::Trigger>("/sr1_mission_executor/stop");
        set_state_client_ = nh_.serviceClient<sr1_state_controller::SetState>("/sr1_state_machine/set_state");
        // Publishers
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/reset_vector", 1 );
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        // Assignment Publishing topic
        updateTopic();

    }
    void ResetMarker::updateTopic()
    {
        robot_model_name_ = robot_name_property_->getStdString();
    }

    // Abstract function from PoseTool
    void ResetMarker::onPoseSet(double x , double y , double theta)
    {
        // This pose is belong to world coordinate no matter what the fixed frame is
        std::string current_frame = context_->getFixedFrame().toStdString();
        ROS_INFO("[ResetMarker]");
        std::cout << "CURRENT FIX FRAME = " << current_frame << std::endl;
        // Create payload
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> input = tf::Stamped<tf::Pose>(
                                        tf::Pose(quat, tf::Point(x, y, 0.0)),
                                        ros::Time::now(),
                                        current_frame
                                    );
        // If Selecting current frame is world frame - we're good to go now
        if (current_frame.find("world") != std::string::npos)
        {
            // Create Pose to spawn robot
            geometry_msgs::PoseStamped goal_pose;
            // Insert value
            tf::poseStampedTFToMsg(input, goal_pose);
            ROS_INFO("[ResetMarker] Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", goal_pose.header.frame_id.c_str(),
            goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
            goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w, theta);

            do_set_robot_pose(goal_pose);
        }
        else
        {
            // We are resetting robot in Map Frame
            // transform it into world frame first then do reset
            // Create TF
            tf::Stamped<tf::Pose> output;
            try
            {
                listener_.transformPose("world", ros::Time(0) , input , current_frame , output);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ROS_ERROR("[ResetMarker] Cannot find corresponding frame - Ignore pose resetting");
                return;
            }
            // Create Pose to spawn robot
            geometry_msgs::PoseStamped goal_pose;
            tf::poseStampedTFToMsg(output, goal_pose);

            ROS_INFO("[ResetMarker] Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", goal_pose.header.frame_id.c_str(),
            goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
            goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w, theta);

            do_set_robot_pose(goal_pose);
        }
    }

    // Set Robot pose !
    void ResetMarker::do_set_robot_pose(geometry_msgs::PoseStamped pose)
    {
        // Guaranteed World Frame ID when arrived
        set_state("stop_mission");
        // Set to stage_ros
        bool stage_success = set_stage_ros_pose(pose.pose);
        // Set to Gazebo
        bool gazebo_success = set_gazebo_ros_pose(pose.pose);

        if (stage_success || gazebo_success){
            ROS_INFO("[ResetMarker] Set pose in simulator successfully");
        }
        else
        {
            ROS_ERROR("[ResetMarker] Cannot set pose in simulator");
        }

        set_initial_worker(pose.pose, pose.header.frame_id);
    }

    bool ResetMarker::set_stage_ros_pose(geometry_msgs::Pose pose)
    {
        stage_ros::SetPose srv;
        // Stage doesn't need model name - pick from the model which has laser(ranger) module (which is only one)
        srv.request.pose = pose;
        bool result = stage_ros_client_.call(srv);
        if (result)
        {
            ROS_INFO("[ResetMarker] Stage Set pose successfully");
            return true;
        }
        else
        {
            ROS_ERROR("[ResetMarker] Stage Failed to call service set pose");
            return false;
        }
    }

    bool ResetMarker::set_gazebo_ros_pose(geometry_msgs::Pose pose)
    {
        gazebo_msgs::SetModelState srv;
        srv.request.model_state.model_name = robot_model_name_;
        srv.request.model_state.pose = pose;
        bool result = gazebo_ros_client_.call(srv);
        if (result)
        {
            ROS_INFO("[ResetMarker] Gazebo Set pose successfully");
            return true;
        }
        else
        {
            ROS_ERROR("[ResetMarker] Gazebo Failed to call service set pose");
            return false;
        }
    }

    bool ResetMarker::set_state(std::string trigger)
    {
        sr1_state_controller::SetState srv;
        srv.request.state = trigger;
        bool result = set_state_client_.call(srv);
        if(result)
        {
            ROS_INFO("[ResetMarker] Set State with trigger : %s", trigger.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("[ResetMarker] Set State with trigger : %s", trigger.c_str());
            return false;
        }
    }

    void ResetMarker::set_initial_worker(geometry_msgs::Pose pose, std::string frame_id)
    {
        // Create PoseStamped
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header.frame_id = frame_id;

        // Transform pose to current map frame
        tf::Stamped<tf::Pose> output;
        tf::Stamped<tf::Pose> world_tf;
        tf::poseStampedMsgToTF(pose_stamped, world_tf);

        try
        {
            listener_.transformPose("map", ros::Time(0) , world_tf , world_tf.frame_id_ , output);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        geometry_msgs::PoseStamped current_map_to_robot_pose;
        tf::poseStampedTFToMsg(output, current_map_to_robot_pose);

        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.pose.pose = current_map_to_robot_pose.pose;
        // initial_pose.pose.pose = pose;

        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = ros::Time::now();

        // Covariance
        initial_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
        initial_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
        initial_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
        std::cout << "[ResetMarker] Set pose for Initial Worker !" << std::endl;
        std::cout << initial_pose << std::endl;
        initial_pose_pub_.publish(initial_pose);
    }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::ResetMarker, rviz::Tool )