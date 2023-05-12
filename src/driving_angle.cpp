#include <vector>
#include <iomanip>
#include <utility>
#include <cmath>

#include <ros/ros.h>
#include <router/router.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>

#define pi 3.14159265358979323846
#define earth_radius 6371.0 // km

// gps dataType
struct GPS{
    double latitude;
    double longitude;

    GPS(double lat, double lon): latitude(lat), longitude(lon) {}
};

struct Vector2 {
    double x, y;

    Vector2(double _lat, double _lon){
        double lat = _lat * pi /180;
        double lon = _lon * pi /180;

        this->x = earth_radius * cos(lat) * cos(lon);
        this->y = earth_radius * cos(lat) * sin(lon);
    }
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

// calculate distance with gps two points
double haversine(double lat1, double lon1, double lat2, double lon2) {
    // convert latitude and longitude to radians
    double lat1_rad = lat1 * pi / 180.0;
    double lon1_rad = lon1 * pi / 180.0;
    double lat2_rad = lat2 * pi / 180.0;
    double lon2_rad = lon2 * pi / 180.0;

    // calculate differences
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    // calculate haversine formula
    double a = pow(sin(dlat / 2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = earth_radius * c * 1000;

    return distance;
}

// update route to next destination
void setNextGpsPoint(Router&router, const GPS&currPoint, GPS&nextPoint, int&nextPointIdx) {
    double distance = haversine(currPoint.latitude, currPoint.longitude, nextPoint.latitude, nextPoint.longitude);

    // inside 1m update next rout path
    if(distance <= 1){
        // arrive goal
        if(++nextPointIdx == router.getCoordinates().size()-1){
            nextPoint.latitude = -1;
            nextPoint.longitude = -1;

            ROS_INFO("here is end point (Latitude: %f, Longitude: %f)", nextPoint.latitude, nextPoint.longitude);
            return;
        }

        std::vector<double> point = router.getCoordinates()[nextPointIdx];
        nextPoint.latitude = point[1];
        nextPoint.longitude = point[0];
    }
}

// calculation angle with (currPoint->nextPoint)vector and (curPoint->tf.x)vector
double angle_calculation(GPS&prevPoint, GPS&currPoint, GPS&nextPoint) {
    if((prevPoint.latitude == currPoint.latitude && prevPoint.longitude == currPoint.longitude) || prevPoint.latitude < 0 || prevPoint.longitude < 0)
        return 0;

    Vector2 prev(prevPoint.latitude, prevPoint.longitude);
    Vector2 curr(prevPoint.latitude, prevPoint.longitude);
    Vector2 next(prevPoint.latitude, prevPoint.longitude);
    // 벡터 a,b와 벡터 a,c를 구하기
    double pToc_x = curr.x - prev.x;
    double pToc_y = curr.y - prev.y;
    double cTon_x = next.x - curr.x;
    double cTon_y = next.y - curr.y;
    // 벡터 a,b와 벡터 a,c의 크기 계산
    double pToc_d = sqrt(pToc_x * pToc_x + pToc_y * pToc_y);
    double cTon_d = sqrt(cTon_x * cTon_x + cTon_y * cTon_y);
    // 벡터 a,b와 벡터 a,c의 내적 계산
    double dot = pToc_x * cTon_x + pToc_y * cTon_y;
    // 라디안으로 각도 계산
    double radian = acos(dot / (pToc_d * cTon_d));
    // 각도를 도로 변환
    return radian * 180 / pi;
}

int main(int argc, char **argv) {
    // create node
    ros::init(argc, argv, "driving_angle_pub");
    ros::NodeHandle nh;
    
    int nextPointIdx = 0;

    Router router;

    GPS prevPoint(-1,-1.000001);
    GPS nextPoint(-123456789,-123456789);
    GPS currPoint(-123456789,-123456789);

    // subscribe GPS
    ros::Subscriber fix_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 10, [&](
        const sensor_msgs::NavSatFix::ConstPtr& msg) {
            currPoint = get_latitude_longitude(msg);
        });

    // publish angle
    ros::Publisher double_pub = nh.advertise<std_msgs::Float64>("/angle", 10);

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("prev point (Latitude: %f, Longitude: %f)", prevPoint.latitude, prevPoint.longitude);
        ROS_INFO("curr point (Latitude: %f, Longitude: %f)", currPoint.latitude, currPoint.longitude);
        ROS_INFO("next point (Latitude: %f, Longitude: %f)", nextPoint.latitude, nextPoint.longitude);

        // init Router
        if(nextPoint.latitude < 0 && nextPoint.longitude < 0 && currPoint.latitude > 0 && currPoint.longitude > 0){
            router = setRouter({currPoint.longitude, currPoint.latitude}, {127.2800, 36.7652});

            nextPoint.latitude = router.getCoordinates()[0][1];
            nextPoint.longitude = router.getCoordinates()[0][0];
            
            // view routing path
            viewRoute(router.getCoordinates());
        }

        // if we have next point continue
        if(nextPoint.latitude > 0 && nextPoint.longitude > 0){
            setNextGpsPoint(router, currPoint, nextPoint, nextPointIdx);

            std_msgs::Float64 msg;
            msg.data = angle_calculation(prevPoint, currPoint, nextPoint);

            double_pub.publish(msg);
        }

        ROS_INFO("distance : %lf", haversine(currPoint.latitude, currPoint.longitude, nextPoint.latitude, nextPoint.longitude));
        ROS_INFO("angle : %lf\n", angle_calculation(prevPoint, currPoint, nextPoint));

        if(haversine(currPoint.latitude, currPoint.longitude, prevPoint.latitude, prevPoint.longitude) >= 0.5)
            prevPoint = currPoint;
        
        rate.sleep();
    }
    return 0;
}