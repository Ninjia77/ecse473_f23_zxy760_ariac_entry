#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Model.h"

std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::Model> mater_type;
std::string test;
void OrderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
    ROS_INFO("products.type is %s",order_vector[0].shipments[0].products[0].type.c_str());
    ROS_INFO("products.type is %f",order_vector[0].shipments[0].products[0].pose.position.x);
}

void bincamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
    if(mater_type.empty())mater_type=msg->models;
    test=mater_type[0].type;
    ROS_INFO("mater_type[0] %s",mater_type[0].type.c_str());
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


   
    int begin_serv_call_succeeded,mater_loc_call_succeeded;
    begin_serv_call_succeeded = begin_client.call(begin_comp);
   

    if(begin_comp.response.success)
    {
        ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    } 
    else 
    {
        ROS_ERROR("Competition service call failed! Goodness Gracious!!");
        ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    }

    getloc.request.material_type=test;
    mater_loc_call_succeeded = mater_loc_client.call(getloc);
    
    order_vector.clear();
    ros::Rate loop_rate(10);
    
    int count=0;
    while(ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(count>10) break;
        count++;
    }
    return 0;
}