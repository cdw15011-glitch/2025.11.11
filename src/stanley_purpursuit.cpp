#include "morai/header.h"

double ego_yaw;
double ego_x = 0.0;
double ego_y = 0.0;
double current_speed = 0.0;
const double L = 3.01;

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

struct Position_path 
{
    double xx;
    double yy;
    double zz;
};
//경로의 한 점을 저장

vector<Position_path> read_path(const string& filename) 
{
    vector<Position_path> path;
    ifstream file(filename);

    double x1, y1, z1;
    while (file >> x1 >> y1 >> z1) {
        path.push_back({x1, y1, z1});
    }

    file.close();
    return path;
}

int find_nearpoint(const vector<Position_path>& path)
{
    int near_point= 0;
    double min_dist = FLT_MAX;
    double dx,dy,dist;

    for (int i = 0; i < path.size(); i++)
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
    return near_point;
}
//nearpoint도 targetpoint처럼 

double compute_stanley_control(const vector<Position_path>& path)
{    
    double dx,dy,dx_ego,dy_ego;

    int near_point = find_nearpoint(path);
       
    dx = path[near_point + 1].xx - path[near_point].xx;
    dy = path[near_point + 1].yy - path[near_point].yy;
    double path_yaw = atan2(dy, dx); //두경로점사이의 기울기방향 , path진행방향

    dx_ego = path[near_point].xx - ego_x;
    dy_ego = path[near_point].yy - ego_y;
    double error_dist = -sin(path_yaw) * dx_ego + cos(path_yaw) * dy_ego;  ;//경로의 점과 차량 사이의 측면거리(가로방향) 오차
   
    double delta = path_yaw - ego_yaw;
    while (delta > M_PI){delta -= 2 * M_PI;}
    while (delta < -M_PI){delta += 2 * M_PI;}
    //핸들의 튐을 방지
   
    double k =1;
    double steering_angle = delta + atan2(k * error_dist , current_speed);     ///조향각 - currentspeed 사용

    cout   << "Speed: " << current_speed << endl
            <<"steering_angle: " << steering_angle<< endl
            <<"error_dist: "<<error_dist<<endl<<endl;

    return steering_angle;
}
double get_look_ahead(double speed) //함수가 이름이 안좋음 getlookahead>>동사로 
{
    // 속도에 비례해서 증가
    double min_lookahead = 1;   // 최소 전방주시거리
    double max_lookahead = 5; // 최대 전방주시거리
    double lookahead = speed * 0.8 + min_lookahead;  // 적당한 비례값
    
    lookahead = clamp(lookahead, min_lookahead, max_lookahead);// 범위를 제한 
    //if (lookahead > max_lookahead) {lookahead = max_lookahead;} 
    //if (lookahead < min_lookahead) {lookahead = min_lookahead;}
    return lookahead;
}

int find_target_point(const vector<Position_path>& path, int near_point, double lookahead_distance)
{   
    double dx,dy,dist;
    int target_point = near_point;

    for (int i = near_point; i < path.size(); i++)
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
    return target_point;
}

double compute_purepursuit_control(const vector<Position_path>& path)
{   
    Position_path target;

    double lookahead_distance = get_look_ahead(current_speed);
        
    int near_point = find_nearpoint(path);
    int target_point = find_target_point(path, near_point,lookahead_distance);
    target = path[target_point];
       
    double x = target.xx - ego_x;
    double y = target.yy - ego_y;

    double target_angle = atan2(y, x); //목표점 방향 각도, 목표점이 있는 절대방향
    //atan2(y, x)는 원점에서 벡터 (x, y)가 가리키는 절대 방향 각도(radians)를 사분면을 고려해 정확하게 반환하는 함수
    //atan(y/x)는 사분면 정보 상실(부호만으로는 어느 쪽 사분면인지 모름)과 x=0에서 0으로 나누기 오류 가능성

    double alpha = target_angle - ego_yaw; //목표점의 각도 - 차량의 진행방향사이의 각도차  
    while (alpha > M_PI){alpha -= 2 * M_PI;}
    while (alpha < -M_PI){alpha += 2 * M_PI;}
    //핸들의 튐을 방지

    double steering_angle = atan2(2.0 * L * sin(alpha), lookahead_distance); //조향각

    cout << "Speed: " << current_speed << " \n "
          <<"lookahead_distance: " << lookahead_distance<< endl
          <<"steering_angle: "<< steering_angle<<endl<<endl;

    return steering_angle;
}

double compute_blend_control(const vector<Position_path>& path){
    double delta_sta = compute_stanley_control(path);
    double delta_pp = compute_purepursuit_control(path);

    double steer_abs = fabs(delta_pp);
    double x = 0.20;   // rad (약 20도)
    double blend = clamp(steer_abs / x, 0.0, 1.0);
    double delta_final = (1.0 - blend) * delta_pp + blend * delta_sta;
    return delta_final;
}

double control_speed(double steering_angle)
{
    double straight_speed=60.0;
    double curve_min_speed = 8.0;

    double abs_steering_angle = fabs(steering_angle);
    double target_steering_angle = 1.0/(1.0 + abs_steering_angle * 1.0); //속도 감소 

    double target_speed = straight_speed*target_steering_angle;

    if (target_speed<=curve_min_speed)
    {
        target_speed = curve_min_speed;
    }

    return target_speed;
}
void publish_ctrlcmd(ros::Publisher& ctrl_pub, const vector<Position_path>& path)
{
    double steering_angle = compute_blend_control(path);

    double target_speed = control_speed(steering_angle);
    

    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = steering_angle; //조향각
    cmd.velocity = target_speed; //차량의 목표 속도.
    cmd.accel = 0.0; //가속도
    cmd.brake = 0.0;
    ctrl_pub.publish(cmd);        
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

    ros::Rate rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        publish_ctrlcmd(ctrl_pub, path);
        rate.sleep();
    }
    return 0;
}
