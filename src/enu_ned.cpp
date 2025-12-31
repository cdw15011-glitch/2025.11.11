//enu>>ned
//euler 사용

#include "ros/ros.h"
#include "morai_msgs/GPSMessage.h"
#include "GeographicLib/UTMUPS.hpp" //시스템 패키지, 좌표변환 도구
#include <iomanip>
#include<cmath>

using namespace std;

const double a=6378137.0;
const double e2 =0.00669437999014;
const double east_offset=313008.5581980090;
const double north_offset = 4161698.6283680100;
const double lattitude_offset=37.5828346072;
const double longitude_offset = 126.8889045862;
const double altitude_offset = 37.1500083995;

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

	cout << "latitude : " << setprecision(8) <<lat << endl;
	cout << "longtitude : " << setprecision(8) <<lon<<endl;
	cout << "altitude : " << setprecision(8) << alt << " \n";

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


	double ned_x =  north;
	double ned_y = east;
	double ned_z = -up;

	cout << "  !!enu to ned!!  " << endl;
	cout << "east : " << setprecision(8) <<ned_x << endl;
	cout << "north : " << setprecision(8) <<ned_y <<endl;
	cout << "up : " << setprecision(8) << ned_z  << " \n\n\n\n";
}

int main(int argc, char **argv){

	ros::init(argc,argv,"wgs84_to_utm");
	//초기화
	ros::NodeHandle nh;
	//이 객체를 통해 rodmaster와 통신하며 서브스크라이버, 접근 가능
	//ros시스템과 연결
	ros::Subscriber sub1 = nh.subscribe<morai_msgs::GPSMessage>("/gps",1000,callback);
	//q사이즈 1000
	//콜백함수에서는 변환및 메시지 출력

	ros::spin();

}
