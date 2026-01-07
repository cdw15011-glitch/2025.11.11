//센서를 후륜축에! >> stanley를 기반으로 하기에 double front_x = ego_x + L * cos(ego_yaw);  
//                                             double front_y = ego_y + L * sin(ego_yaw);
#include "morai/header.h"

//고정
double ego_yaw;
double ego_x= 0.0;
double ego_y = 0.0;
double current_speed = 0.0;
double steering_angle = 0.0;
double accel = 0.0;
double brake = 0.0;
const double L = 3.0;  // 차량 휠베이스
double max_curvature = 0.0;

//튜닝값
double Kp = 0.15;  
double Kd = 0.05;
const double target_speed =60/3.6;
const double min_speed = 15/3.6;
float curve_standard = 0.03;
float ld = 0.35;

struct Position_path {
    double xx;
    double yy;
    double zz;
    double curvature;
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
        path.push_back({x, y, z,0.0});
    return path;
}

//------------------------------------------------------------//
//----------------stanley,lookahead-----------------------//
//------------------------------------------------------------//

double get_look_ahead(double speed)
{
    double min_lookahead = 1.0;
    double max_lookahead = 10.0;
    double lookahead = speed * ld + min_lookahead;
    return clamp(lookahead, min_lookahead, max_lookahead);
}

int find_nearpoint(const vector<Position_path>& path)
{
    double dx,dy,dist;
    static int last_near_point = 0;
    int near_point= last_near_point;
    double min_dist = DBL_MAX;

    int start_point = max(0, last_near_point - 10);
    int end_point   = min((int)path.size() - 1, last_near_point + 30);
   
    double front_x = ego_x + L * cos(ego_yaw);  
    double front_y = ego_y + L * sin(ego_yaw);

    for (int i = start_point; i <= end_point; i++)
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

    if (near_point >= path.size() - 2)return path.size() - 2;

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

//----------------------------------------------//
//----------------속도 로직-----------------------//
//----------------------------------------------//
void compute_curvature(vector<Position_path>& path)
{
    for(int i =0; i<path.size()-2; i++)
    {
        double dx1 = path[i+1].xx - path[i].xx;
        double dy1 = path[i+1].yy - path[i].yy;
        double dx2 = path[i+2].xx - path[i+1].xx;
        double dy2 = path[i+2].yy - path[i+1].yy;

        double yaw1 = atan2(dy1, dx1);
        double yaw2 = atan2(dy2, dx2);

        double k = fabs(yaw2 - yaw1); //곡률
        if (k > M_PI) k = 2*M_PI - k;
        path[i+1].curvature = k;    
    }

    path[0].curvature = path[1].curvature;  //첫점의 곡률을 두번쨰 점의 곡률로!
    path.back().curvature = path[path.size()-2].curvature; // path.back() : 마지막 점, 마지막점의 곡률은 계산이 안됨>>전전점으로  
}

void getMaxCurvature(const vector<Position_path>& path, int near_point, int ld_point, double& max_curvature) //참조로 값 변경 
{
    double max_kappa = 0.0;

    int end_idx = min((int)path.size()-1, ld_point); //near~ldpoint까지의 점을 탐색  

    for (int i = near_point; i <= end_idx; ++i)
    {
        if (path[i].curvature > max_kappa)
            max_kappa = path[i].curvature; // ldpoint까지의 점중에서 가장 큰 곡률을 저장 
    }
    max_curvature = max_kappa;
}

void compute_stanley_control(const vector<Position_path>& path)
{
    double lookahead_distance = get_look_ahead(current_speed);

    int near_point = find_nearpoint(path);
    int ld_point     = find_ld_point(path, near_point,lookahead_distance);

    steering_angle = compute_stanley_ld(path, ld_point);

    getMaxCurvature(path, near_point, ld_point, max_curvature);

    cout  <<" lookahead : "<<lookahead_distance<<endl
          << " near_point : " << near_point<<endl
          << " max_curvature : "<<max_curvature <<endl<<endl;
}

void compute_pid(double current_speed,double target_speed_pid,double& accel, double& brake)
{ //참조>> 값의 변경을 위함, 별명, 복사x,함수 안에서 바뀐 값이 외부에 적용)

    static double prev_error = 0.0;    

    double error = target_speed_pid - current_speed;
    double p_error = Kp*error;
    double d_error = Kd*((error - prev_error)/0.02); //50hz >> 0.02
    prev_error = error;

    double pid = p_error + d_error;

    if (pid > 0) { //가속
        accel = min(pid, 1.0); //min(a, b)는 a와 b 중에서 작은 값을 반환.
        brake = 0.0;
    }
    else { //감속
        accel = 0.0;
        brake = min(-pid, 1.0); //current: 16 target: 14 >>>>>>>>>  pid speed<0 감속 진행 음수>양수 브레이크 값 사용 가능    
    }
}

bool publish_stop_ctrlcmd(const vector<Position_path>& path,ros::Publisher& ctrl_pub) {
    double dx = path.back().xx - ego_x;
    double dy = path.back().yy - ego_y;
    double dist = sqrt(dx*dx + dy*dy);

    if (dist < 1.0) {
        morai_msgs::CtrlCmd stop_cmd;
        stop_cmd.longlCmdType = 1;
        stop_cmd.velocity = 0.0;
        stop_cmd.steering = 0.0;
        stop_cmd.accel = 0.0;
        stop_cmd.brake = 1.0;
        ctrl_pub.publish(stop_cmd);
        return true;
    }
    return false;
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

    compute_curvature(path);

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        compute_stanley_control(path);

        double target_speed_pid = target_speed;
        if (max_curvature > curve_standard)target_speed_pid = min_speed;

        compute_pid(current_speed,target_speed_pid,accel,brake);
        if (publish_stop_ctrlcmd(path, ctrl_pub))break;
        publish_ctrlcmd(ctrl_pub);
        rate.sleep();
    }
    return 0;
}

// double compute_curvature(const vector<Position_path>& path, int near_point)
// {
//    

//     double x1 = path[near_point+1].xx, y1 = path[near_point+1].yy;
//     double x2 = path[near_point+2].xx, y2 = path[near_point+2].yy;
//     double x3 = path[near_point+3].xx, y3 = path[near_point+3].yy;
//    
//     double a = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
//     double b = sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2));
//     double c = sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1));

//     if (a*b*c == 0) return 0.0;

//     return 2 * fabs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / (a*b*c);
//     //삼각형을 외접하는 원의 반지름 R과 삼각형 넓이A 공식
//     //넓이 A는 외적공식을 활용
//     //2>>4로 변경하는것이 일반적(값의 증폭,민감도향상을 위해서)
// }
//>>계산량이 많아 복잡, 느릴수 있음

