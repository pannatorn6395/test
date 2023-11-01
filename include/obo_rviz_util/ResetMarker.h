/*********************************
* ResetMarker.h
* Marker for reseting robot position and simulation position
* Based on OBO Worker Navigation system
* Author : Theppasith N. <theppasith.n@hiveground.com>
* 13 - Mar - 2018
*********************************/

#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// Services call
#include <stage_ros/SetPose.h>
#include <sr1_state_controller/SetState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Trigger.h>
// Pose tools inside rviz
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>
namespace rviz
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    class ResetMarker: public PoseTool
    {
        Q_OBJECT
        public:
            ResetMarker();
            virtual ~ResetMarker();
            virtual void onInitialize();

        protected:
            virtual void onPoseSet(double x, double y, double theta);

        private Q_SLOTS:
            void updateTopic();

        private:
            ros::NodeHandle nh_;
            ros::Publisher  pub_;
            ros::Publisher  initial_pose_pub_;
            ros::ServiceClient stage_ros_client_;
            ros::ServiceClient gazebo_ros_client_;
            // ros::ServiceClient executor_client_;
            ros::ServiceClient set_state_client_;
            tf::TransformListener listener_;
            // Toolsbox setup
            StringProperty* robot_name_property_;
            std::string robot_model_name_;
            // Functions
            void do_set_robot_pose(geometry_msgs::PoseStamped pose);
            bool set_stage_ros_pose(geometry_msgs::Pose pose);
            bool set_gazebo_ros_pose(geometry_msgs::Pose pose);
            bool set_executor_stop();
            bool set_state(std::string trigger);
            void set_initial_worker(geometry_msgs::Pose pose, std::string frame_id);
    };
}
