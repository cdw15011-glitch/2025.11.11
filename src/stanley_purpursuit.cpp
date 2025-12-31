//센서를 후륜축에! >> stanley를 기반으로 하기에 double front_x = ego_x + L * cos(ego_yaw);  
//                                             double front_y = ego_y + L * sin(ego_yaw);


#include "morai/header.h"

using namespace std;

double ego_yaw;
double ego_x= 0.0;
double ego_y = 0.0;
double current_speed = 0.0;
double steering_angle = 0.0;
const double L = 2.9;  // 차량 휠베이스
const float w = 0.7;
const float target_speed = 60;
double accel = 0.0;
double brake = 0.0;
double Kp = 0.5;  
double Kd = 0.1;

struct Position_path {
    double xx;
    double yy;
    double zz;
};

double quaternion_to_yaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    return atan2(siny_cosp, cosy_cosp);
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

void callback_enu(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ego_x = msg->pose.position.x;
    ego_y = msg->pose.position.y;
}

void callback_velocity(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
{
    current_speed = msg->velocity.x;
}

vector<Position_path> read_path(const string& filename)
{
    vector<Position_path> path;
    ifstream file(filename);
    double x, y, z;
    while (file >> x >> y >> z)
        path.push_back({x, y, z});
    return path;
}

struct PIDController
{
    double kp;
    double kd;
    double prev_error = 0.0;
};
double get_look_ahead(double speed)
{
    double min_lookahead = 1.0;
    double max_lookahead = 10.0;
    double lookahead = speed * 0.8 + min_lookahead;
    return clamp(lookahead, min_lookahead, max_lookahead);
}

int find_nearpoint(const vector<Position_path>& path)
{
    double dx,dy,dist;
    static int last_near_point = 0;
    int near_point= last_near_point;
    double min_dist = DBL_MAX; //DBL_MAX는 double타입의 최댓값, FLT_MAX = float 타입의 최대값
    
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

int find_ld_point(const vector<Position_path>& path, int near_point, double ld) //stanley에 ld 적용 >> ld와 가까운 거리에 있는 point 위치  
{
    double front_x = ego_x + L * cos(ego_yaw);  
    double front_y = ego_y + L * sin(ego_yaw);

    int ld_point = near_point;
    double min_dist = DBL_MAX;

    for (int i = near_point; i < path.size(); i++)
    {
        double dx = path[i].xx - front_x;
        double dy = path[i].yy - front_y;
        double dist = sqrt(dx*dx + dy*dy);

        if (dist >ld)
        {
            ld_point = i;
            break;
        }
    }
    return ld_point;
}

double compute_stanley_ld(const vector<Position_path>& path, int ld_point)
{
    if (ld_point + 1 >= path.size()) return 0.0;

    double front_x = ego_x + L * cos(ego_yaw);
    double front_y = ego_y + L * sin(ego_yaw);

    double dx = path[ld_point + 1].xx - path[ld_point].xx;
    double dy = path[ld_point + 1].yy - path[ld_point].yy;
    double path_yaw = atan2(dy, dx);

    double dx_ego = path[ld_point].xx - front_x;
    double dy_ego = path[ld_point].yy - front_y;

    double error_dist = -sin(path_yaw) * dx_ego + cos(path_yaw) * dy_ego;

    double delta = path_yaw - ego_yaw;
    while (delta > M_PI){delta -= 2 * M_PI;}
    while (delta < -M_PI){delta += 2 * M_PI;}

    if (current_speed < 1) current_speed = 1;

    float k = 2.0;

    return delta + atan2(k * error_dist , current_speed);
}

void compute_stanley_control(const vector<Position_path>& path)
{
    double lookahead_distance = get_look_ahead(current_speed);

    int near_point = find_nearpoint(path);
    int ld_point     = find_ld_point(path, near_point,lookahead_distance);

    steering_angle = compute_stanley_ld(path, ld_point);

    cout << " Speed: " << current_speed<< endl
         << " lookahead: "<<lookahead_distance<<endl
         << " Steering: " << steering_angle<< endl<< endl;
}

void compute_pid(double current_speed,double target_speed,double& accel, double& brake){

    static double prev_error = 0.0;     
    double error = target_speed - current_speed;

    double p_error = Kp*error;
    double d_error = Kd*((error - prev_error)/0.02); //50hz >> 0.02
    prev_error = error;

    double pid_speed = p_error + d_error;

    if (pid_speed > 0) {
        accel = min(pid_speed, 1.0); //min(a, b)는 a와 b 중에서 작은 값을 반환.
        brake = 0.0;
    } 
    else {
        accel = 0.0;
        brake = min(-pid_speed, 1.0); 
    }

}
void publish_ctrlcmd(ros::Publisher& ctrl_pub)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.steering = steering_angle;
    cmd.accel = accel;
    cmd.brake = brake;
    ctrl_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stanley_purepursuit_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, callback_yaw);
    ros::Subscriber enu_sub = nh.subscribe("/enu_pose", 1, callback_enu);
    ros::Subscriber velocity_sub = nh.subscribe("/Ego_topic", 1, callback_velocity);

    ros::Publisher ctrl_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);

    vector<Position_path> path = read_path("/home/autonav/cyg_ws/src/morai/path.txt");

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        compute_stanley_control(path);
        compute_pid(current_speed,target_speed,accel,brake);
        publish_ctrlcmd(ctrl_pub);
        rate.sleep();
    }
    return 0;
}


// int find_nearpoint(const vector<Position_path>& path)
// {
//     int near_point = 0;
//     double min_dist = FLT_MAX;
//     for (int i = 0; i < path.size(); i++)
//     {
//         double dx = path[i].xx - ego_x;
//         double dy = path[i].yy - ego_y;
//         double dist = sqrt(dx*dx + dy*dy);
//         if (dist < min_dist)
//         {
//             min_dist = dist;
//             near_point = i;
//         }
//     }
//     return near_point;
// }

// double compute_curvature(const vector<Position_path>& path, int near_point)
// {
//     if (near_point + 7 >= path.size()) return 0.0;

//     double x1 = path[near_point+3].xx, y1 = path[near_point+3].yy;
//     double x2 = path[near_point+5].xx, y2 = path[near_point+5].yy;
//     double x3 = path[near_point+7].xx, y3 = path[near_point+7].yy;
//    //
//     double a = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
//     double b = sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2));
//     double c = sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1));

//     if (a*b*c == 0) return 0.0;

//     return 2 * fabs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / (a*b*c);
//     //삼각형을 외접하는 원의 반지름 R과 삼각형 넓이A 공식
//     //넓이 A는 외적공식을 활용
//     //2>>4로 변경하는것이 일반적(값의 증폭,민감도향상을 위해서)
// }

// double compute_stanley_control(const vector<Position_path>& path, int near_point)
// {
//     if (near_point + 1 >= path.size()) return 0.0;

//     double dx,dy,dx_ego,dy_ego;

//     double front_x = ego_x + L * cos(ego_yaw);  
//     double front_y = ego_y + L * sin(ego_yaw);

//     dx = path[near_point+1].xx - path[near_point].xx;
//     dy = path[near_point+1].yy - path[near_point].yy;
//     double path_yaw = atan2(dy, dx); //두경로점사이의 기울기방향 , path진행방향

//     dx_ego = path[near_point].xx - front_x;
//     dy_ego = path[near_point].yy - front_y;
//     double error_dist = -sin(path_yaw) * dx_ego + cos(path_yaw) * dy_ego;  ;//경로의 점과 차량 사이의 측면거리(가로방향) 오차

//     double delta = path_yaw - ego_yaw;
//     while (delta > M_PI){delta -= 2 * M_PI;}
//     while (delta < -M_PI){delta += 2 * M_PI;}

//     if (current_speed < 0.1) current_speed = 0.1;

//     float k =2.0;
//     return delta + atan2(k * error_dist , current_speed);     ///조향각 - currentspeed 사용
// }

// double compute_purepursuit_control(const vector<Position_path>& path, int target_point, double lookahead_distance)
// {
//     double dx = path[target_point].xx - ego_x;
//     double dy = path[target_point].yy - ego_y;
//     double target_angle = atan2(dy, dx);

//     double alpha = target_angle - ego_yaw;
//     while (alpha > M_PI) alpha -= 2*M_PI;
//     while (alpha < -M_PI) alpha += 2*M_PI;

//     return atan2(2.0 * L * sin(alpha), lookahead_distance);
// }


// double compute_curvature(const vector<Position_path>& path, int ld_point)
// {
//     if (ld_point + 2 >= path.size()) return 0.0;

//     double dx1 = path[ld_point+1].xx - path[ld_point].xx;
//     double dy1 = path[ld_point+1].yy - path[ld_point].yy;
//     double dx2 = path[ld_point+2].xx - path[ld_point+1].xx;
//     double dy2 = path[ld_point+2].yy - path[ld_point+1].yy;

//     double yaw1 = atan2(dy1, dx1);
//     double yaw2 = atan2(dy2, dx2);

//     double k = fabs(yaw2 - yaw1);
//     if (k > M_PI) k = 2*M_PI - k;

//     return k;
// }