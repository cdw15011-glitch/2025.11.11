//조향각 설정 알고리즘
//look ahead distance필요
//목표점과 차량의 상대각 alpha 필요>>조향각
//전방주시거리가 멀어지면 안정적으로 가지만 경로와의 오차가 커진다. 짧아지면 와리가리
//운동방정식과 경로의 지오메트리 방정식 필요?
//차량의 현재위치 알아야함
//path.txt에서의 각점을 global path에 저장하여 위치이동
//목표점까지 가려면 차량은 원을 그리면서 회전.
//→ Pure Pursuit은 이 원의 곡률(curvature) 를 계산해서 조향각을 구함.

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "morai_msgs/CtrlCmd.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <iomanip>
#include <cmath>
using namespace std;

double ego_yaw; //차량의 yaw값, 차량이 지금 바라보는 각도(진행방향)
//쿼터니안 to yaw

double ego_x = 0.0; //차량의 현재위치
double ego_y = 0.0;
bool enu_received = false;
//ENU 좌표(또는 적어도 한 번의 /enu_pose 메시지)를 받았는지 여부를 표시

nav_msgs::Path global_path;
bool path_received = false;
//path.txt

const double lookahead_distance = 6.0; //전방 주시거리 
const double L = 3.01; //// 차량의 휠베이스 (앞뒤 바퀴 간 거리)
geometry_msgs::PoseStamped target_point; //waypoint



//쿼터니안>> yaw,  yaw는 상대적 회전값이고, heading은 자북 기준의 절대 방향
double quaternionToYaw(double x, double y, double z, double w)
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
    ego_yaw  = quaternionToYaw(
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
    enu_received = true;
}

void callback_path(const nav_msgs::Path::ConstPtr& msg)
{
    global_path = *msg;      // 받은 path를 global_path에 저장, 포인터 -> 객체로 역참조
    path_received = true;
}

int main(int argc, char **argv)
{
    
    ros::init(argc,argv, "purpursit_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 10, callback_yaw);
    ros::Subscriber enu_sub = nh.subscribe("/enu_pose", 10, callback_enu);
    ros::Subscriber path_sub = nh.subscribe("/pub_path", 10, callback_path);

    ros::Publisher ctrl_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 10);

    ros::Rate rate(10);

    int start_point=0;

    while(ros::ok())
    {
        bool found = false;

        if(path_received&&enu_received) //path_received와 enu_received 가 true일때만 성립 
        {
                for (int i = start_point; i < global_path.poses.size(); i++)
            {
                    geometry_msgs::PoseStamped pose1 = global_path.poses[i];
                     double dx = pose1.pose.position.x - ego_x;
                    //경로(global_path)에 있는 i번째 점의 x좌표
                    double dy = pose1.pose.position.y - ego_y;
                    double dist = sqrt(dx*dx + dy*dy);
                    //목표지점과 차량의 거리


                    if (dist > lookahead_distance)
                     {
                        target_point = pose1;
                        found = true;
                        start_point=i;
                         break;
                     }
            }
                //전방주시거리와 목표지점간의 차이를 비교하여 목표거리>주시거리 이면 조향각 설정
                //현재 차량과의 가장가까운 점을 waypoint로 잡기!

              if (found) //found = true; 일때만 성립
                 {
                        double x = target_point.pose.position.x - ego_x;
                        double y = target_point.pose.position.y - ego_y;

                        double target_angle = atan2(y, x); //목표점 방향 각도, 목표점이 있는 절대방향
                        double alpha = target_angle - ego_yaw; //목표점의 각도 - 차량의 진행방향사이의 각도차  
                        if (alpha > M_PI)
                            {alpha -= 2 * M_PI;}
                        if (alpha < -M_PI)
                            {alpha += 2 * M_PI;}
                        //핸들의 튐을 방지 


                        double steering_angle = atan2(2.0 * L * sin(alpha), lookahead_distance); //조향각 

                        morai_msgs::CtrlCmd cmd;
                        cmd.steering = steering_angle; //조향각
                        cmd.velocity = 3.0; //차량의 목표 속도.
                        cmd.accel = 0.25; //가속도 
                        cmd.brake = 0.0;

                        ctrl_pub.publish(cmd);
                }
        }
        ros::spinOnce();
        rate.sleep();
    }
return 0;

}
//인덱스를 뽑아서 각 장애물에 인덱스번호를 매겨서 주행
//움직이는 물체는 라이더와 카메라 사용 >> 그때그떄 다름 
//차량의 뒷축에 imu센서와 gps센서 잡기 
//주시거리를 동적으로 설정해야함 가장기본적으로는 속도에 의해 주시거리 설정  
//주시거리 튜닝(변수투닝)은 주로 1차식으로 튜닝 
//주로 클래스보다 구조체(한 노드안에 변수)를 사용
//path한번에 받아서 진행 
//ld 생ㄱ각하며 코드 짜기 