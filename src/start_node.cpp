#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Model.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "ik_service/PoseIK.h"
#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"


std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> mater_type;
std::vector<sensor_msgs::JointState> joint_state;
int count=0;
void OrderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
    ROS_INFO("products.type is %s",order_vector[0].shipments[0].products[0].type.c_str());
    ROS_INFO("products.type is %f",order_vector[0].shipments[0].products[0].pose.position.x);
}
void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state.push_back(*msg);
}
void bincamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
    mater_type.push_back(*msg);
    ROS_INFO("POSExyz%f",mater_type.back().pose.position.x);
}

int main(int argc, char **argv)
{
    //init node
    ros::init(argc, argv, "start_node");
    ros::NodeHandle nh;

    std_srvs::Trigger begin_comp;
    osrf_gear::GetMaterialLocations getloc;
    trajectory_msgs::JointTrajectory desired;
    double T_pose[4][4], T_des[4][4];
    double q_pose[6], q_sol[8][6];

    ros::ServiceClient begin_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient mater_loc_client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");

    std_srvs::SetBool my_bool_var;
    my_bool_var.request.data = true;
    ros::Publisher command_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 100);

    ros::Subscriber order_sub = nh.subscribe<osrf_gear::Order>("/ariac/orders", 10,OrderCallback);
    ros::Subscriber camera_bin_sub = nh.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 10,bincamCallback);
    ros::Subscriber joint_state_sub= nh.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states",10,jointstateCallback);

    int begin_serv_call_succeeded;
    begin_serv_call_succeeded = begin_client.call(begin_comp);
   
    if(begin_comp.response.success)
    {
        ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    } 
    else 
    {
        ROS_ERROR("Competition service call failed!");
        ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    }
 
    order_vector.clear();
    ros::Rate loop_rate(10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped tfStamped;
    geometry_msgs::PoseStamped part_pose, goal_pose;

    trajectory_msgs::JointTrajectory ur10_trajectory;

    //ros::AsyncSpinner spinner(1); 
    //spinner.start();
    while(ros::ok)
    {
        ros::spinOnce();
        if(mater_type.size()>0)
        {
            ROS_INFO("running!size%d",mater_type[0].models.size());
            getloc.request.material_type=mater_type.back().models[0].type;
            goal_pose.pose=mater_type[0].models[0].pose;
            mater_loc_client.call(getloc);
            //if(!getloc.response.storage_units.empty())
            //goal_pose.pose=getloc.response.storage_units[0].
            try 
            {
                tfStamped = tfBuffer.lookupTransform("arm1_base_link", "bin4_frame",ros::Time(0.0), ros::Duration(1.0));
                ROS_INFO("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
                tfStamped.child_frame_id.c_str());
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            tf2::doTransform(part_pose, goal_pose, tfStamped);
            goal_pose.pose.position.z += 0.10; 
            goal_pose.pose.orientation.w = 0.707;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;
            goal_pose.pose.orientation.z = 0.0;
            ROS_INFO("service type %s", getloc.response.storage_units.back().unit_id.c_str());
            ROS_INFO("test");
            //T_des[0][3] = goal_pose.pose.position.x;
            //T_des[1][3] = goal_pose.pose.position.y;
            //T_des[2][3] = goal_pose.pose.position.z;
            T_des[0][3] = 0.5;
            T_des[1][3] = 0;
            T_des[2][3] = 0;
            ROS_INFO("XYZ%f,%f,%f",T_des[0][3],T_des[1][3],T_des[2][3]);
            T_des[3][3] = 1.0;

            T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
            T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
            T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
            T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

            int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_sol);
            ROS_INFO("solutions:%d",num_sols);
        }
        
        if(!joint_state.empty())
        {
            ROS_INFO("velocity is %f",joint_state[0].velocity[0]);

            q_pose[0] = joint_state.back().position[1];
            q_pose[1] = joint_state.back().position[2];
            q_pose[2] = joint_state.back().position[3];
            q_pose[3] = joint_state.back().position[4];
            q_pose[4] = joint_state.back().position[5];
            q_pose[5] = joint_state.back().position[6];
            ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

            ur10_trajectory.header.seq = count++;
            ur10_trajectory.header.stamp = ros::Time::now();
            ur10_trajectory.header.frame_id = "/world";

            ur10_trajectory.joint_names.clear();
            ur10_trajectory.joint_names.push_back("linear_arm_actuator_joint");
            ur10_trajectory.joint_names.push_back("shoulder_pan_joint");
            ur10_trajectory.joint_names.push_back("shoulder_lift_joint");
            ur10_trajectory.joint_names.push_back("elbow_joint");
            ur10_trajectory.joint_names.push_back("wrist_1_joint");
            ur10_trajectory.joint_names.push_back("wrist_2_joint");
            ur10_trajectory.joint_names.push_back("wrist_3_joint");

            ur10_trajectory.points.resize(2);

            ur10_trajectory.points[0].positions.resize(ur10_trajectory.joint_names.size());
            for (int i = 0; i < ur10_trajectory.joint_names.size(); i++) 
                for (int j = 0; j < joint_state.back().name.size(); j++) 
                    if (ur10_trajectory.joint_names[i] == joint_state.back().name[j]) 
                    {
                        ur10_trajectory.points[0].positions[i] = joint_state.back().position[j];
                        break;
                    }
            ur10_trajectory.points[0].time_from_start = ros::Duration(0.0);

            ur10_trajectory.points[1].positions.resize(ur10_trajectory.joint_names.size());
            ur10_trajectory.points[1].positions[0] = joint_state.back().position[1];
            for (int i = 0; i < 6; i++) {
                ur10_trajectory.points[1].positions[i + 1] = q_sol[0][i];
            }
            ur10_trajectory.points[1].time_from_start = ros::Duration(1.0);
            command_pub.publish(ur10_trajectory);
        }
        loop_rate.sleep();
    }
    return 0;
}
