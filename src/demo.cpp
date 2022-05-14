#include "ros/ros.h"
#include <xarm_api/xarm_driver.h>
#include<math.h>

class naist_xarm{
    private:
            ros::NodeHandle nh; 
            ros::Publisher sleep_pub_ ;
            ros::ServiceClient client;
            ros::ServiceClient motion_ctrl_client_ ;
            ros::ServiceClient set_mode_client_;
            ros::ServiceClient set_state_client_ ;
            ros::ServiceClient go_home_client_ ;
            ros::ServiceClient move_lineb_client_ ;
            ros::ServiceClient move_servoj_client_ ;
            xarm_msgs::SetAxis set_axis_srv_;
            xarm_msgs::SetInt16 set_int16_srv_;
            xarm_msgs::Move move_srv_;
            xarm_msgs::Move srv;
            std_msgs::Float32 sleep_msg;
            float alt_x, alt_y;
            
    public:
            naist_xarm();
            int motion ();
            int xarm_mode_set();
            int WS_Set_pose(float x,float y,float z);
            int change_Cartesian_position(xarm_msgs::Move srv, ros::ServiceClient client,float x,float y,float z,float ax,float ay,float az);
            int home_pose(xarm_msgs::Move srv, ros::ServiceClient client);
};  

naist_xarm::naist_xarm(){
    sleep_pub_ = nh.advertise<std_msgs::Float32>("/xarm/sleep_sec", 1);
    motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
    set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
    set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
    go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
    move_servoj_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servoj");
    move_lineb_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_lineb");
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    sleep_msg.data = 1.0;
};

int naist_xarm::xarm_mode_set(){
        if(motion_ctrl_client_.call(set_axis_srv_)){
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }

    set_int16_srv_.request.data = 0;
    if(set_mode_client_.call(set_int16_srv_)){
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service set_mode");
        return 1;
    }  

    set_int16_srv_.request.data = 0;
    if(set_state_client_.call(set_int16_srv_)){
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service set_state");
        return 1;
    }
}

int naist_xarm::WS_Set_pose(float x,float y,float z){
    float deg = 30 ;
    alt_x=x*cos(deg*(M_PI/180)) - y*sin(deg*(M_PI/180))  ;
    alt_y=x*sin(deg*(M_PI/180)) + y*cos(deg*(M_PI/180))  ;
    return alt_x,alt_y,z;
}

int naist_xarm::home_pose(xarm_msgs::Move srv, ros::ServiceClient client){
    srv.request.mvvelo = 20.0 / 57.0;
    srv.request.mvacc = 1000;
    srv.request.mvtime = 0;
    if(client.call(srv)){
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service go home");
        return 1;
    }
    return 0;
}

int naist_xarm::change_Cartesian_position(xarm_msgs::Move srv, ros::ServiceClient client,float x, float y, float z,float ax,float ay,float az){
    naist_xarm::WS_Set_pose(x,y,z);
    srv.request.pose = {alt_x, alt_y, z, ax, ay, az};
    srv.request.mvvelo = 100;
    srv.request.mvacc = 1000;
    srv.request.mvtime = 0;
    srv.request.mvradii = 20;
   
    if(client.call(srv)){
        ROS_INFO("%s\n", srv.response.message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service change_Cartesian_position");
        return 1;
    }
    return 0;
}

int naist_xarm::motion(){
    nh.setParam("/xarm/wait_for_finish", true); // return after motion service finish
    
        naist_xarm::xarm_mode_set();    
   /*
    if(home_pose(move_srv_, go_home_client_) == 1) 
        return 1; 
    
    nh.setParam("/xarm/wait_for_finish", false); 
    
    sleep_pub_.publish(sleep_msg);
    */
   
    nh.setParam("/xarm/wait_for_finish", true);
    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, -200, 300, -3.14, 0, 0) == 1)
        return 1;
    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, -200, 130, -3.14, 0, 0) == 1)
        return 1;

        ros::Duration(10).sleep();

    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, -200, 300, -3.14, 0, 0) == 1)
        return 1;
    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, 200, 300, -3.14, 0, 0) == 1)
        return 1;
    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, 200, 130, -3.14, 0, 0) == 1)
        return 1;

        ros::Duration(10).sleep();

    if(change_Cartesian_position(move_srv_, move_lineb_client_,400, 200, 300, -3.14, 0, 0) == 1)
        return 1;

    nh.setParam("/xarm/wait_for_finish", false); 
    nh.setParam("/xarm/wait_for_finish", true);   

}

int main(int argc, char **argv){
    ros::init(argc, argv, "xarm_move_test");
    naist_xarm xArm;
    xArm.motion();
}
