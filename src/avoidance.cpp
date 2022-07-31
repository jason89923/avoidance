/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>
#include <vector>
using namespace std;
#define RAD2DEG(x) ((x)*180. / M_PI)

class Group {
   public:
    vector<pair<int, int>> index_group;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    Group() {
        index_group.clear();
        index_group.push_back(move(pair<int, int>(0, 9)));
        index_group.push_back(move(pair<int, int>(10, 19)));
        index_group.push_back(move(pair<int, int>(20, 29)));
        index_group.push_back(move(pair<int, int>(691, 700)));
        index_group.push_back(move(pair<int, int>(701, 710)));
        index_group.push_back(move(pair<int, int>(711, 719)));
        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Group::scanCallback, this);
        pub = n.advertise<std_msgs::String>("/collision", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> average;
        for (int i = 0; i < index_group.size(); i++) {
            double sum = 0;
            int counter = 0;
            for (int j = index_group[i].first; j <= index_group[i].second; j++) {
                if (scan->ranges[j] == 0)
                    counter++;
                sum += scan->ranges[j];
            }
            if (sum == 0)
                average.push_back(0);
            else
                average.push_back(sum / (index_group[i].second - index_group[i].first - counter + 1));
        }

        int num_of_collision = 0;
        for (int i = 0; i < average.size(); i++) {
            if (average[i] <= 0.3) {
                num_of_collision++;
            }
        }

        std_msgs::String msg;
        msg.data = "false";
        if (num_of_collision >= 2)
            msg.data = "true";
            
        pub.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance");
    Group group;

    ros::spin();

    return 0;
}
