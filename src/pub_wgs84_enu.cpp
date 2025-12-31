#include "ros/ros.h"
#include "morai_msgs/GPSMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include <iomanip>
#include<cmath>

using namespace std;

const double a=6378137.0;
const double e2 =0.00669437999014;

const double lattitude_offset=37.238838359501933 ;
const double longitude_offset = 126.772902206454901;
const double altitude_offset =  0.000000000000000;

ros::Publisher enu_pub;
//콜백함수 및 메인함수 에서도 사용하기에 전역으로 설정

void callback(const morai_msgs::GPSMessage::ConstPtr& msg)
{
	double lat = msg ->latitude;
	//msg ->latitude : 메시지안에있는 LATITUDE,위도 값임
	//double타입의 변수로 lat에 복사
	//sub한 ros메시지에서 위도 값을 추출해 lat변수에 저장
	double lon = msg ->longitude;
	//longitude, 경도값임
	double alt= msg ->altitude;
	//altitude,고도값

	double lat_x = lat*M_PI/180;
	double lon_y = lon*M_PI/180;
	//위도경도>>라디안

 	double N = a/sqrt(1-e2*pow(sin(lat_x),2));
 	double X = (N+alt)*cos(lat_x)*cos(lon_y);
	double Y = (N+alt)*cos(lat_x)*sin(lon_y);
	double Z = ((1-e2)*N+alt)*sin(lat_x);
	//wgs84 >> ecef변환

	double lat0 = lattitude_offset * M_PI / 180.0;
	double lon0 = longitude_offset * M_PI / 180.0;
	double h0   = altitude_offset;
	//기준점의 위도경도>>라디안

	double N0 = a / sqrt(1 - e2 * pow(sin(lat0), 2));
	double X0 = (N0 + h0) * cos(lat0) * cos(lon0);
	double Y0 = (N0 + h0) * cos(lat0) * sin(lon0);
	double Z0 = ((1 - e2) * N0 + h0) * sin(lat0);
	//기준점의 wgs84 >> ecef변환

	double del_X = X-X0;
	double del_Y=Y-Y0;
	double del_Z=Z-Z0;

	double east = -sin(lon0)*del_X + cos(lon0)*del_Y;
	double north =-sin(lat0)*cos(lon0)*del_X-sin(lat0)*sin(lon0)*del_Y+cos(lat0)*del_Z;
	double up = cos(lat0)*cos(lon0)*del_X+cos(lat0)*sin(lon0)*del_Y+sin(lat0)*del_Z;

	geometry_msgs::PoseStamped enu_pose;
	//enu_pose라는 변수생성
	enu_pose.pose.position.x = east;
	enu_pose.pose.position.y = north;
	enu_pose.pose.position.z = up;
	//계산한 ENU 좌표(east, north, up)를
	//메시지의 위치 부분(pose.position)에 차례로 넣음
	enu_pub.publish(enu_pose);
	// .publish()를 하면, 지금 만든 enu_pose 메시지가 ROS 시스템 전체에 발송.
	// 다른 노드가 /enu_pose를 구독(subscribe)하고 있으면
	// → 그 노드가 이 좌표 데이터를 즉시 받게 됨.
}

int main(int argc, char **argv){

	ros::init(argc,argv,"wgs84_to_utm");
	//초기화
	ros::NodeHandle nh;
	//이 객체를 통해 rodmaster와 통신하며 서브스크라이버, 접근 가능
	//ros시스템과 연결

	ros::Subscriber sub = nh.subscribe<morai_msgs::GPSMessage>("/gps", 10, callback);
    enu_pub = nh.advertise<geometry_msgs::PoseStamped>("/enu_pose", 10);

	ros::spin();

}