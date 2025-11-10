#include "ros/ros.h"
#include "nav_msgs/Path.h"
//Path는 여러 PoseStamped를 묶어 경로를 표현하는 메시지 타입.
#include "geometry_msgs/PoseStamped.h"
//하나의 점(자세 포함된 좌표)을 나타내는 메시지, Path는 이걸 여러 개 묶은 구조
#include <fstream>
//파일 입출력용 표준 C++ 헤더. 여기서는 std::ifstream으로 path.txt를 읽음.
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/pub_path", 10);

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";  // ENU 좌표계 기준이라면 "map" 사용
    //이 경로(path)의 모든 좌표는 map 좌표계 기준이라는 뜻    

    // 파일 열기
    std::ifstream file("/home/autonav/cyg_ws/src/2025.10.30/path.txt");

    double x, y, z;
    //파일에서 읽어올 좌표 변수.
    while (file >> x >> y >> z)
    {
        geometry_msgs::PoseStamped pose1; // pose: 맵의 한 점
        pose1.pose.position.x = x;
        pose1.pose.position.y = y;
        pose1.pose.position.z = z;
        path_msg.poses.push_back(pose1);
        //Path 메시지의 벡터에 해당 점 추가,
        //.push_back(pose) : 리스트 끝에 방금 만든 pose를 하나 추가
        //push_back이란 > 리스트(vector)의 맨 뒤에 새로운 데이터를 하나 추가
    }

    file.close();
    //파일 닫기

    ros::Rate rate(10);
    while (ros::ok())
    {
        path_pub.publish(path_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}