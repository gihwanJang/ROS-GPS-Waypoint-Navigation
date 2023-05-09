#include <vector>
#include <iomanip>
#include <utility>

#include <ros/ros.h>
#include <router/router.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>

// gps dataType
struct GPS{
    double latitude;
    double longitiude;

    GPS(double lat, double lon): latitude(lat), longitiude(lon) {}
};
// set routing function
Router setRouter(std::pair<double, double> s, std::pair<double, double> e) {
    std::string api_key = "5b3ce3597851110001cf6248058d8f01fd6e490b9d412f14e6f771ec";
    std::vector<double> start = {s.first, s.second};
    std::vector<double> end = {e.first, e.second};

    return Router(api_key, start, end);
}
// print routing path
void viewRoute(std::vector<std::vector<double>> coordinates) {
    std::cout << "경로 정보" << std::endl;
    for (std::vector<double> coord : coordinates) {
        std::cout << "경도: ";
        std::cout << std::fixed << std::setprecision(6) << coord[0];
        std::cout << ", 위도: ";
        std::cout << std::fixed << std::setprecision(6) << coord[1] << "\n"; 
    }
}
// subscribe current gps
GPS get_latitude_longitude(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    return GPS(msg->latitude, msg->longitude);
}
// update route to next destination
void setNextGpsPoint(const Router&router, const GPS&currPoint, GPS&nextPoint, int&nextPointIdx) {
    // 만약 idx가 0이면 처음값 초기화

    // 만약 idx가 마지막 index이면 nextPoint랑 nextPointIdx모두 -1로 초기화

    // 이외에는 currPoint가 nextPoint의 반경 1m이내에 접급하면 nextPoint, nextPointIdx갱신
}
// calculation angle with (currPoint->nextPoint)vector and (curPoint->tf.x)vector
double angle_calculation() {
    // currPoint, nextPoint를 이용해 vector계산

    // currPoint, tf.x를 이용해 vector계산

    // 두 벡터의 사이각을 산출 아마 벡터 내적? 이용하면 될 듯
}

int main(int argc, char **argv) {
    // create node
    ros::init(argc, argv, "driving_angle_pub");
    ros::NodeHandle nh;
    
    // init router
    Router router = setRouter({127.281351, 36.763353}, {127.28158, 36.766249});
    int nextPointIdx = 0;
    GPS nextPoint(-123456789,-123456789);
    GPS currPoint(-123456789,-123456789);

    // subscribe GPS
    ros::Subscriber fix_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 10, [&](
        const sensor_msgs::NavSatFix::ConstPtr& msg) {
            currPoint = get_latitude_longitude(msg);
        });
    // publish angle
    ros::Publisher double_pub = nh.advertise<std_msgs::Float64>("/angle", 10);
    
    // view routing path
    viewRoute(router.getCoordinates());

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("curr point (Latitude: %f, Longitude: %f)", currPoint.latitude, currPoint.longitiude);
        ROS_INFO("next point (Latitude: %f, Longitude: %f)", nextPoint.latitude, nextPoint.longitiude);

        // if we have next point continue
        if(nextPoint.latitude > 0 && nextPoint.longitiude > 0){
            // std_msg::Float64 msg;
            // msg.data = angle_calculation();

            // double_pub.publish(msg);

            // setNextGpsPoint(router, currPoint, nextPoint, nextPointIdx);
        }
        
        rate.sleep();
    }
    return 0;
}