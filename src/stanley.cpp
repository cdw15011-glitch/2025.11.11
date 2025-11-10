#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "morai_msgs/CtrlCmd.h"
#include "geometry_msgs/PoseStamped.h"
#include "morai_msgs/EgoVehicleStatus.h"
//속도 sub
#include <cmath>
#include <iostream>
#include <fstream>
//파일 입출력 헤더. 파일을 열고(ifstream/ofstream) 읽거나 쓸 때 사용.
#include <vector>
//가변 크기 배열 컨테이너. 경로처럼 점(Point)을 여러 개 저장하고 싶을 때 사용.
#include <string>
#include <cfloat>   // FLT_MAX사용 위함 
#include <algorithm> //clamp사용 위함
using namespace std;

double ego_yaw;
double ego_x = 0.0;
double ego_y = 0.0;
double current_speed = 0.0;


double quaternion_to_yaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    //sin(ψ) * cos(θ)
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    //cos(ψ) * cos(θ)
    return atan2(siny_cosp, cosy_cosp);
    //ψ = atan2(sinψ*cosθ, cosψ*cosθ) = atan2(sinψ, cosψ) =yaw(ψ)
}
void callback_yaw(const sensor_msgs::Imu::ConstPtr& msg)
{
    ego_yaw  = quaternion_to_yaw(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
 }
 //ego_yaw에 yaw값 저장

void callback_enu(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ego_x = msg->pose.position.x;
    ego_y = msg->pose.position.y;
}
void callback_velocity(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    current_speed = msg->velocity.x;  // 단위: m/s
}

struct Position_path {
    double xx;
    double yy;
    double zz;
};
//경로의 한 점을 저장

vector<Position_path> read_path(const string& filename) {
    vector<Position_path> path;
    ifstream file(filename);

    double x1, y1, z1;
    while (file >> x1 >> y1 >> z1) {
        path.push_back({x1, y1, z1});
    }

    file.close();
    return path;
}

void stanley_control(ros::Publisher& ctrl_pub, const vector<Position_path>& path)
{     
    double dx,dy,dx_ego,dy_ego;
    int near_point= 0;
    double min_dist = FLT_MAX;

    for (int i = 0; i < path.size(); i++)
        {
            dx = path[i].xx - ego_x;
            dy = path[i].yy - ego_y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                near_point = i;
            }
        }
    dx = path[near_point + 1].xx - path[near_point].xx;
    dy = path[near_point + 1].yy - path[near_point].yy;
    double path_yaw = atan2(dy, dx); //두경로점사이의 기울기방향 , path진행방향 

    dx_ego = path[near_point].xx - ego_x;
    dy_ego = path[near_point].yy - ego_y;
    double error_dist = -sin(path_yaw) * dx_ego + cos(path_yaw) * dy_ego;  ;//경로의 점과 차량 사이의 측면거리(가로방향) 오차 

    double delta = ego_yaw-path_yaw;
    if (delta > M_PI){delta -= 2 * M_PI;}
    if (delta < -M_PI){delta += 2 * M_PI;}

    if (current_speed==0){current_speed=0.1;}

    double steering_angle = delta + atan2(0.8 * error_dist , current_speed);     ///조향각 - currentspeed 사용 

    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = steering_angle; //조향각
    cmd.velocity = 20; //차량의 목표 속도.
    cmd.accel = 0.0; //가속도
    cmd.brake = 0.0;
    ctrl_pub.publish(cmd);        
    cout << "Speed: " << current_speed << endl
          <<"steering_angle: " << steering_angle<< endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "stanley_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, callback_yaw);
    ros::Subscriber enu_sub = nh.subscribe("/enu_pose", 1, callback_enu);
    ros::Subscriber velocity_sub = nh.subscribe("/Ego_topic", 1, callback_velocity);

    ros::Publisher ctrl_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

    vector<Position_path> path = read_path("/home/autonav/cyg_ws/src/2025.11.06/path.txt");

    ros::Rate rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        stanley_control(ctrl_pub, path);
        rate.sleep();
    }
    return 0;
       
}