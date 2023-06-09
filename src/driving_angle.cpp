#include <vector>
#include <iomanip>
#include <utility>
#include <cmath>

#include <ros/ros.h>
#include <router/router.hpp>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

#define pi 3.14159265358979323846
#define earth_radius 6371.0

#define GO 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

#define ROS_RATE 3
#define TURN_SPEED 110
#define DRIVE_SPEED 150
#define TURN_TIME 30
#define TURN_ANGLE 9
#define LANE_MAX 100
#define OBJECT_DISTANCE 1.3
#define IGNORE_VALUE 10
#define CORRECT_TIME 3

// gps dataType
struct GPS{
    double latitude;
    double longitude;

    GPS(double lat, double lon): latitude(lat), longitude(lon) {}
    GPS(){}
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

// lane detect callback
void lane_callback(const std_msgs::Float64::ConstPtr& lane_pub, float&lane_vel){
    lane_vel = lane_pub->data;
    
    if(lane_vel >= LANE_MAX)        lane_vel = LANE_MAX;
    if(lane_vel <= (-1 * LANE_MAX)) lane_vel = (-1 * LANE_MAX);
}

// laser topic callback
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan, float&min_distance) {
    // Set an initial value for min_distance
    min_distance = std::numeric_limits<float>::max();
    
    // Check the validity of scan data
    if (scan->ranges.size() == 0) {
        ROS_WARN("Empty laser scan data received.");
        return;
    }
    
    // Iterate through the laser scan ranges
    for (int i = 0; i < scan->ranges.size(); ++i) {
        float distance = scan->ranges[i];

        if(distance < 0.2)
            continue;

        if (std::isfinite(distance) && distance < min_distance) {
            min_distance = distance;
        }
    }
}

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
    if(std::isnan(msg->latitude))
        return GPS(-1,-1);
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
void setNextGpsPoint(Router&router, const GPS&currPoint, GPS&nextPoint, int&nextPointIdx, std::vector<int>&directs, int achieve_bound) {
    double distance = haversine(currPoint.latitude, currPoint.longitude, nextPoint.latitude, nextPoint.longitude);

    // inside achieve bound update next rout path
    if(distance <= achieve_bound){
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
    Vector2 curr(currPoint.latitude, currPoint.longitude);
    Vector2 next(nextPoint.latitude, nextPoint.longitude);
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
    // 각도를 도로 변환하여 0부터 360도 사이로 조정
    double degree = radian * 180 / pi;
    if (pToc_x * cTon_y - pToc_y * cTon_x < 0)
        degree = 360 - degree;
    // 각도를 도로 변환
    return degree;
}

// routing direction
void setDirection(std::vector<std::vector<double>>&coordinates, std::vector<int>&directs){
    double angle;
    GPS prev, curr, next;

    for(int i = 1; i < coordinates.size() - 1; ++i){
        if(i == 1){
            directs.push_back(GO);
            directs.push_back(GO);
        }
        else{
            prev = GPS(coordinates[i-1][0], coordinates[i-1][1]);
            curr = GPS(coordinates[i][0], coordinates[i][1]);
            next = GPS(coordinates[i+1][0], coordinates[i+1][1]);

            angle = angle_calculation(prev, curr, next);

            if(25 < angle && angle < 90)
                directs.push_back(LEFT);
            else if(270 < angle && angle < 335)
                directs.push_back(RIGHT);
            else
                directs.push_back(GO);
        }
    }

    for(int i = 0; i < directs.size(); ++i){
        if(directs[i] == GO)
            ROS_INFO("%d : GO", i + 1);
        if(directs[i] == LEFT)
            ROS_INFO("%d : LEFT", i + 1);
        if(directs[i] == RIGHT)
            ROS_INFO("%d : RIGHT", i + 1);
    }
}

// calculate cmd vel
void correction_median(geometry_msgs::Twist&msg, float median_vel, float prev, int&timer){
    if(median_vel * prev > 0)
        ++timer;
    else
        timer = 0;

    if(std::abs(prev) - std::abs(median_vel) > 100 || timer >= 3)
        return;

    msg.angular.x += median_vel/180 * 8;

    if(timer == 2)
        msg.angular.x -= median_vel/180 * 10 - median_vel/abs(median_vel) * 10;

    ROS_INFO("median : %d", msg.angular.x);
}

int main(int argc, char **argv) {
    // create node
    ros::init(argc, argv, "driving_nav_pub");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg;
    std::vector<std::vector<double>> coordinates;
    std::vector<int> directs;
    Router router;
    float min_distance = 0;
    float median_vel_prev = 0;
    float median_vel = 0;
    int nextPointIdx = 0;
    int timer_direct = 0;
    int timer_median = 0;

    GPS nextPoint(-123456789,-123456789);
    GPS currPoint(-123456789,-123456789);

    // subscribe lazer
    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(
        &laser_callback, _1, boost::ref(min_distance)
        ));

    // subscribe GPS
    ros::Subscriber fix_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, [&](
        const sensor_msgs::NavSatFix::ConstPtr& msg) {
            currPoint = get_latitude_longitude(msg);
        });

    ros::Subscriber sub_lane_pub = nh.subscribe<std_msgs::Float64>("/lane_pub", 1, boost::bind(&lane_callback, _1, boost::ref(median_vel)));

    //publisher
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(ROS_RATE);
    while (ros::ok()) {
        ros::spinOnce();

        ROS_INFO("curr point (Latitude: %f, Longitude: %f)", currPoint.latitude, currPoint.longitude);
        ROS_INFO("next point (Latitude: %f, Longitude: %f)", nextPoint.latitude, nextPoint.longitude);

        // init Router
        if(nextPoint.latitude < 0 && nextPoint.longitude < 0 && currPoint.latitude > 0 && currPoint.longitude > 0){
            router = setRouter({currPoint.longitude, currPoint.latitude}, {std::stod(argv[1]), std::stod(argv[2])});

            nextPoint.latitude = router.getCoordinates()[0][1];
            nextPoint.longitude = router.getCoordinates()[0][0];

            coordinates = router.getCoordinates();
            setDirection(coordinates, directs);
            
            // view routing path
            viewRoute(router.getCoordinates());
        }
        
        // navigation
        if(nextPoint.latitude > 0 && nextPoint.longitude > 0){
            setNextGpsPoint(router, currPoint, nextPoint, nextPointIdx, directs, std::stoi(argv[3]));

            if(directs[nextPointIdx-1] == GO){
                ROS_INFO("Go Straight");

                msg.linear.x = DRIVE_SPEED;
                msg.angular.x = 0;

                correction_median(msg, median_vel, median_vel_prev, timer_median);
            }
            if(directs[nextPointIdx-1] == LEFT){                                        // If, we shoud turn left or right action
                if(0 < timer_direct && timer_direct < TURN_TIME){                       // Fist, go straight during TURN_TIME.
                    ROS_INFO("Go Left , timer_direct : %d", timer_direct);
                    
                    msg.linear.x = TURN_SPEED;
                    msg.angular.x = (-1 * TURN_ANGLE);
                    ++timer_direct;
                }
                else{                                                                   // Second, Turn Action do it.
                    if(timer_direct >= TURN_TIME){
                        timer_direct = 0;
                        directs[nextPointIdx - 1] = GO;
                    } else
                        ++timer_direct;
                    
                    ROS_INFO("Go Straight , timer_direct : %d", timer_direct);

                    msg.linear.x = DRIVE_SPEED;
                    msg.angular.x = 0;
                    //correction_median(msg, median_vel);
                }
            }
            if(directs[nextPointIdx-1] == RIGHT){
                if(0 < timer_direct && timer_direct < TURN_TIME){
                    ROS_INFO("Go Right , timer_direct : %d", timer_direct);
                    
                    msg.linear.x = TURN_SPEED;
                    msg.angular.x = TURN_ANGLE;
                    ++timer_direct;
                }
                else{
                    if(timer_direct >= TURN_TIME){
                        timer_direct = 0;
                        directs[nextPointIdx - 1] = GO;
                    } else
                        ++timer_direct;
                    
                    ROS_INFO("Go Straight , timer_direct : %d", timer_direct);

                    msg.linear.x = DRIVE_SPEED;
                    msg.angular.x = 0;
                    //correction_median(msg, median_vel);
                }
            }
            if(min_distance <= OBJECT_DISTANCE){                                        // If near from object, stop the robot.
                ROS_INFO("Stop");

                msg.linear.x = 0;
                msg.angular.x = 0;
                if(0 < timer_direct)                                                    // If we was turing left or right, timer is not increase.
                    --timer_direct;
            }
        }

	if(abs(median_vel_prev - median_vel) <= IGNORE_VALUE){
        	median_vel_prev = median_vel;
	}
        
        ROS_INFO("distance to next point : %lf", haversine(currPoint.latitude, currPoint.longitude, nextPoint.latitude, nextPoint.longitude));
        ROS_INFO("Object detected within %f meters\n", min_distance);
        pub_cmd_vel.publish(msg);
        
        rate.sleep();
    }
    return 0;
}
