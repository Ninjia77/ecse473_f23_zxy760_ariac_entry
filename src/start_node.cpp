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


std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> mater_type;
int sum=0;

void OrderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
    ROS_INFO("products.type is %s",order_vector[0].shipments[0].products[0].type.c_str());
    ROS_INFO("products.type is %f",order_vector[0].shipments[0].products[0].pose.position.x);
}

void bincamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
    mater_type.push_back(*msg);
    sum++;
}

int main(int argc, char **argv)
{
    //init node
    ros::init(argc, argv, "start_node");
    ros::NodeHandle nh;

    std_srvs::Trigger begin_comp;
    osrf_gear::GetMaterialLocations getloc;

    ros::ServiceClient begin_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient mater_loc_client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    
    std_srvs::SetBool my_bool_var;
    my_bool_var.request.data = true;

    ros::Subscriber order_sub = nh.subscribe<osrf_gear::Order>("/ariac/orders", 10,OrderCallback);
    ros::Subscriber camera_bin_sub = nh.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_1", 10,bincamCallback);

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
    while(ros::ok)
    {
        ros::spinOnce();
        ROS_INFO("SIZE%d",sum);
        if(!mater_type.empty())
        {
            getloc.request.material_type=mater_type[sum-1].models[0].type;
            mater_loc_client.call(getloc);

            try 
            {
                tfStamped = tfBuffer.lookupTransform("arm1_base_link", "bin4_frame",ros::Time(0.0), ros::Duration(1.0));
                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
                tfStamped.child_frame_id.c_str());
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            //part_pose.pose = mater_type[sum-1].models[0].pose;
            goal_pose.pose.position.z += 0.10; 
            goal_pose.pose.orientation.w = 0.707;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;
            goal_pose.pose.orientation.z = 0.0;

            tf2::doTransform(part_pose, goal_pose, tfStamped);

            ROS_INFO("service type %s", getloc.response.storage_units[0].unit_id.c_str());
            ROS_WARN("part_pose");
            ROS_WARN("parr");
        }
        loop_rate.sleep();
    }
    return 0;
}
