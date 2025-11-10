#include "ros/ros.h"
#include "morai_msgs/GPSMessage.h"
#include "GeographicLib/UTMUPS.hpp" //시스템 패키지, 좌표변환 도구
#include <iomanip>

using namespace std;

const double east_offset=313008.5581980090;
const double north_offset = 4161698.6283680100;

void callback(const morai_msgs::GPSMessage::ConstPtr& msg)
{
	double lat = msg ->latitude;
	//msg ->latitude : 메시지안에있는 LATITUDE,위도 값임
	//double타입의 변수로 lat에 복사
	//sub한 ros메시지에서 위도 값을 추출해 lat변수에 저장
	double lon = msg ->longitude;
	double alt= msg ->altitude;
	
	cout << "latitude : " << setprecision(8) <<lat << endl;
	cout << "longtitude : " << setprecision(8) <<lon<<endl;
	cout << "altitude : " << setprecision(8) << alt << " \n";

	double x,y;
	int zone;
    bool north = true;  // 기본값으로 true 설정

	GeographicLib::UTMUPS::Forward(lat, lon, zone, north,x, y);
	//Forward라는 위도경도 > utm으로 변환
	//Reverse를 통해 utm >>위도경도

	 double X=x-east_offset;
	 double Y=y-north_offset;
	 //오차 제거

	cout << setprecision(8) << "lattitude > x : " << X <<endl;
	cout << setprecision(8) << "longitude > y : " << Y <<" \n\n\n\n";
}


int main(int argc, char **argv){

	ros::init(argc,argv,"wgs84_to_utm");
	//초기화
	ros::NodeHandle nh;
	//이 객체를 통해 rodmaster와 통신하며 서브스크라이버, 접근 가능
	//ros시스템과 연결
	ros::Subscriber sub1 = nh.subscribe<morai_msgs::GPSMessage>("/gps",1000,callback);

	//콜백함수에서는 변환및 메시지 출력

	ros::spin();

}
