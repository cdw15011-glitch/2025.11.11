#include "morai/header.h"

double ego_yaw;
double ego_x = 0.0;
double ego_y = 0.0;
double current_speed = 0.0;
double steering_angle =0;

const double L = 3.01; //센서를 차량의 후륜축에 설정하였기에 l만큼 더해준다.

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

// int find_nearpoint(const vector<Position_path>& path)
// {
//     int near_point= 0;
//     double min_dist = FLT_MAX;
//     double dx,dy,dist;

//     double front_x = ego_x + L * cos(ego_yaw);  
//     double front_y = ego_y + L * sin(ego_yaw);

//     for (int i = 0; i < path.size(); i++)
//     {
//         dx = path[i].xx - front_x;
//         dy = path[i].yy - front_y;
//         dist = sqrt(dx * dx + dy * dy);
//         if (dist < min_dist)
//         {
//             min_dist = dist;
//             near_point = i;
//         }
//     }  
//     return near_point;
// }
int find_nearpoint(const vector<Position_path>& path)
{
    double dx,dy,dist;
    static int last_near_point = 0;
    int near_point= last_near_point;
    double min_dist = FLT_MAX;
    
    double front_x = ego_x + L * cos(ego_yaw);  
    double front_y = ego_y + L * sin(ego_yaw);

    for (int i = last_near_point; i < path.size(); i++)
    {
        dx = path[i].xx - front_x;
        dy = path[i].yy - front_y;
        dist = sqrt(dx * dx + dy * dy);

        if (dist < min_dist)
        {
            min_dist = dist;
            near_point = i;
        }
    }  
    last_near_point = near_point;
    return near_point;
}

// double control_speed(double steering_angle)
// {
//     double straight_speed = 30.0;     // 직선 최고속도
//     double min_curve_speed = 10.0;     // 코너 최소속도
//     double sensitivity = .0;         // 조향각 민감도

//     double abs_steer = fabs(steering_angle);
//     double scale = 1.0 / (1.0 + abs_steer * sensitivity);

//     double target_speed = straight_speed * scale;

//     if (target_speed < min_curve_speed)
//         target_speed = min_curve_speed;

//     return target_speed;
// }

void compute_stanley_control(const vector<Position_path>& path)
{    
    double dx,dy,dx_ego,dy_ego;
    double front_x = ego_x + L * cos(ego_yaw);  
    double front_y = ego_y + L * sin(ego_yaw);
    int near_point = find_nearpoint(path);
       
    dx = path[near_point + 1].xx - path[near_point].xx;
    dy = path[near_point + 1].yy - path[near_point].yy;
    double path_yaw = atan2(dy, dx); //두경로점사이의 기울기방향 , path진행방향

    dx_ego = path[near_point].xx - front_x;
    dy_ego = path[near_point].yy - front_y;
    double error_dist = -sin(path_yaw) * dx_ego + cos(path_yaw) * dy_ego;  //경로의 점과 차량 사이의 측면거리(가로방향) 오차
   
    double delta = path_yaw - ego_yaw;
    while (delta > M_PI){delta -= 2 * M_PI;}
    while (delta < -M_PI){delta += 2 * M_PI;}
    //핸들의 튐을 방지
   
    if (current_speed==0){current_speed=0.1;}
 
    int k =2;
    //k가 커지면 lateral error(가로 오차)에 더 민감해짐, 코너 진입이 빨라짐, 좌우 흔들림(oscillation)도 약간 증가 가능
    //steering 반응이 전체적으로 빨라짐

    steering_angle = delta + atan2(k * error_dist , current_speed);     ///조향각 - currentspeed 사용
    cout   << "Speed: " << current_speed << endl
            <<"steering_angle: " << steering_angle<< endl
            <<"error_dist: "<<error_dist<<endl<<endl;
}

void publish_ctrlcmd(ros::Publisher& ctrl_pub)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = steering_angle; //조향각
    // cmd.velocity = control_speed(steering_angle); //차량의 목표 속도.
    cmd.velocity = 30;
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

    vector<Position_path> path = read_path("/home/autonav/cyg_ws/src/morai/path.txt");

    ros::Rate rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        compute_stanley_control(path);
        publish_ctrlcmd(ctrl_pub);
        rate.sleep();
    }
    return 0;
}