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
const double L = 3.01;
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

double get_look_ahead(double speed) //함수가 이름이 안좋음 getlookahead>>동사로 
{
    // 속도에 비례해서 증가
    float min_ld = 1;   // 최소 전방주시거리
    float max_ld = 5; // 최대 전방주시거리
    //타입이 이상...
    float lookahead = speed * 0.8 + min_ld;  // 적당한 비례값
    
    lookahead = clamp(lookahead, min_ld, max_ld);// 범위를 제한 
    //if (lookahead > max_ld) {lookahead = max_ld;} 
    //if (lookahead < min_ld) {lookahead = min_ld;}
    
    return lookahead;
}

void pure_pursuit_control(ros::Publisher& ctrl_pub, const vector<Position_path>& path)
{    //스네이크 카멜 표기
    //tab잘쓰기!!!!! 
    Position_path target;

    double dx, dy, dist;
    double lookahead_distance = get_look_ahead(current_speed);
    double min_dist = FLT_MAX;
    
    int i = 0;
    int near_point= 0;

    cout << "Speed: " << current_speed << " \n "
          <<"lookahead_distance: " << lookahead_distance<< endl;

        //차량의 가장 가까운 포인트 파악
        //첫번쨰 for문 이후 dist로 바뀌어서 nearpoint  변경가능  

    for (i  = 0; i < path.size(); i++)
    {
        dx = path[i].xx - ego_x;
        dy = path[i].yy - ego_y;
        dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            near_point = i;
        }
    }
        //차량의 현재위치 찾는 코드


    int target_point = near_point;
            //차량의 앞쪽점만 보도록 객체 설정  

    for (i = near_point; i < path.size(); i++)
    {
        dx = path[i].xx - ego_x;
        dy = path[i].yy - ego_y;
        dist = sqrt(dx * dx + dy * dy);
        if (dist > lookahead_distance)
        {
            target_point =i;
            break;
        }
    }
    //waypoint 설정

    target = path[target_point];
       
    double x = target.xx - ego_x;
    double y = target.yy - ego_y;

    double target_angle = atan2(y, x); //목표점 방향 각도, 목표점이 있는 절대방향
    //atan2(y, x)는 원점에서 벡터 (x, y)가 가리키는 절대 방향 각도(radians)를 사분면을 고려해 정확하게 반환하는 함수
    //atan(y/x)는 사분면 정보 상실(부호만으로는 어느 쪽 사분면인지 모름)과 x=0에서 0으로 나누기 오류 가능성

    double alpha = target_angle - ego_yaw; //목표점의 각도 - 차량의 진행방향사이의 각도차  
    if (alpha > M_PI){alpha -= 2 * M_PI;}
    if (alpha < -M_PI){alpha += 2 * M_PI;}
    //핸들의 튐을 방지


    double steering_angle = atan2(2.0 * L * sin(alpha), lookahead_distance); //조향각

    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = steering_angle; //조향각
    cmd.velocity = 20; //차량의 목표 속도.
    cmd.accel = 0.0; //가속도
    cmd.brake = 0.0;
    ctrl_pub.publish(cmd);         
   
}

int main(int argc, char **argv)
{
   
    ros::init(argc,argv, "purpursit_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, callback_yaw);
    ros::Subscriber enu_sub = nh.subscribe("/enu_pose", 1, callback_enu);
    ros::Subscriber velocity_sub = nh.subscribe("/Ego_topic", 1, callback_velocity);

    ros::Publisher ctrl_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

    vector<Position_path> path = read_path("/home/autonav/cyg_ws/src/2025.11.06/path.txt");

   
    ros::Rate rate(1000);//최대 50hz
    while(ros::ok())
    {
        ros::spinOnce();
        pure_pursuit_control(ctrl_pub, path);
        rate.sleep();
    }
    return 0;
   
}

//stanley기반으로 대회를 나감 (엄청 발달된 STANLEY)
//stanley 의 흔들거림을 어떻게 해걀할것인가?>> STANLEY는 어쩔수 없이 흔들거림이 발생함 
//이는 stANLEY와 흔들림없는 PUREPURSIT의 장점을 합쳐서 만드는 것을 아이디어만 구상해와라!
//k계수 동적으로!>>아마두 다다음주 과제 
//곡률을 계산하는 공식이 있음 
//FOR문을 I로 계산하는 공식은 다르게 만들기 >> 비효율일수도 있음 point가 많으면 비효울적임 

//멈추게 하는것은 ctrltype이 1으로 해야함!!
//stanley 완성해오기 
//타입을 잘 생각해보기 double?int?
//전역이 많아지면 다른 변수에도 연향이 가기에 가능하면 지역변수마니쓰기 
//변수명 의미있게 작성
// 함수명도 의미있게 작성!!
//stop할때는 index에서의 값을 통해 속도를 줄ㅇ기. (마지막점 근처에서 속도줄이게하기) 
//1함수 1기능 
//is it true >> is_find_first_waypoint(이런식으로 점찾기는 is로 시작)
//디버깅을 위한 한줄을 주석으로 만들엇을떄의 실행을 가능하게 >> 즉 매개변수로 했을떄의 오류가 없도록, 매개변수는 꼐산값이 들어가지않게! 
//FLT_MaX(내부 아주큰 쓰레기 값)
//함수이름은 동사를 넣어라(약속)