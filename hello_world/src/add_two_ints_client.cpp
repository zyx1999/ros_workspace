#include "ros/ros.h"
#include "hello_world/AddTwoInts.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "add_two_ints_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<hello_world::AddTwoInts>("add_two_ints");
    hello_world::AddTwoInts srv;
    srv.request.a = 3;
    srv.request.b = 4;
    if(client.call(srv)){
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    return 0;
}