#include "morai/header.h"

double ego_yaw;
double ego_x = 0.0;
double ego_y = 0.0;
const double L = 3.0;
double current_speed = 0.0;
double steering_angle = 0.0;

double quaternion_to_yaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
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
    current_speed = msg->velocity.x;  // 단위: m/s (사용중인 시뮬/환경에 따라 단위 확인)
}

struct Position_path {
    double xx;
    double yy;
    double zz;
};

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

double get_look_ahead(double speed)
{
    double min_lookahead = 1.0;   // 최소 전방주시거리 (m)
    double max_lookahead = 5.0;   // 최대 전방주시거리 (m)
    double lookahead = speed * 0.5 + min_lookahead;  // 속도 기반 비례
    lookahead = clamp(lookahead, min_lookahead, max_lookahead);
    return lookahead;
}

// int find_nearpoint(const vector<Position_path>& path)
// {
//     int near_point = 0;
//     double min_dist = FLT_MAX;

//     for (int i = 0 ; i < path.size(); i++)
//     {
//         double dx = path[i].xx - ego_x;
//         double dy = path[i].yy - ego_y;
//         double dist = sqrt(dx*dx + dy*dy);

//         if (dist < min_dist) {
//             min_dist = dist;
//             near_point = i;
//         }
//     }
//     return near_point;
// }

// // }
int find_nearpoint(const vector<Position_path>& path)
{
    double dx,dy,dist;
    static int last_near_point = 0;
    int near_point= last_near_point;
    double min_dist = DBL_MAX;

    int start = max(0, last_near_point - 10);
    int end   = min((int)path.size() - 1, last_near_point + 30);
    
    double front_x = ego_x + L * cos(ego_yaw);  
    double front_y = ego_y + L * sin(ego_yaw);

    for (int i = start; i < end; i++)
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

int find_target_point(const vector<Position_path>& path, int near_point, double lookahead_distance)
{
    int target_point = near_point;
    for (int i = near_point; i < (int)path.size(); i++)
    {
        double dx = path[i].xx - ego_x;
        double dy = path[i].yy - ego_y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist > lookahead_distance)
        {
            target_point = i;
            break;
        }
    }
    return target_point;
}

void compute_purepursuit_control(const vector<Position_path>& path)
{
    double lookahead_distance = get_look_ahead(current_speed);

    int near_point = find_nearpoint(path);
    int target_point = find_target_point(path, near_point, lookahead_distance);

    double x = path[target_point].xx - ego_x;
    double y = path[target_point].yy - ego_y;

    double target_angle = atan2(y, x);

    double alpha = target_angle - ego_yaw;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    steering_angle = atan2(2.0 * L * sin(alpha), lookahead_distance);

    cout << "Current speed (sensor): " << current_speed << " \n"
         << "lookahead_distance: " << lookahead_distance << " \n"
         << "steering_angle: " << steering_angle << "\n\n";
}

void publish_ctrlcmd(ros::Publisher& ctrl_pub)
{
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering = steering_angle;
    // 여기서 속도를 control_speed로 결정
    // cmd.velocity = control_speed(steering_angle);
    cmd.velocity = 20;
    cmd.accel = 0.0;
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

    vector<Position_path> path = read_path("/home/autonav/cyg_ws/src/morai/path.txt");

    ros::Rate rate(50); // 50 Hz
    while(ros::ok())
    {
        ros::spinOnce();
        compute_purepursuit_control(path);
        publish_ctrlcmd(ctrl_pub);
        rate.sleep();
    }
    return 0;
}