#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>

#include <iostream>
#include <string>
#include <vector>
using namespace std;

#define window_size 21
#define obstacle_size 5
#define top_distance_stop 0.2
#define top_distance_slow 0.6
#define bottom_distance_stop 0.25
#define bottom_distance_slow 0.6
#define top_corner_distance_stop 0.20
#define bottom_corner_distance_stop 0.20



enum Position {
    top,
    top_R,
    top_L,
    bottom,
    bottom_R,
    bottom_L
};

class Group {
   private:
    vector<int> index_group[6];
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

   public:
    Group() {
        vector<int> temp;
        for (int i = 0; i < 720; i++) {
            if (i >= 0 && i <= 84) {
                temp.push_back(i);
            }

            if (i >= 635 && i <= 719) {
                index_group[top].push_back(i);
            }

            if (i >= 300 && i <= 419) {
                index_group[bottom].push_back(i);
            }

            if (i >= 60 && i <= 179) {
                index_group[top_L].push_back(i);
            }

            if (i >= 540 && i <= 659) {
                index_group[top_R].push_back(i);
            }

            if (i >= 195 && i <= 284) {
                index_group[bottom_L].push_back(i);
            }

            
            if (i >= 435 && i <= 524) {
                index_group[bottom_R].push_back(i);
            }



        }

        for (int i = 0; i < temp.size(); i++) {
            index_group[top].push_back(temp[i]);
        }

        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Group::scanCallback, this);
        pub = n.advertise<std_msgs::Int16MultiArray>("/collision", 1);
    }

    void filter(vector<double>& array, const vector<int>& index_array, const sensor_msgs::LaserScan::ConstPtr& scan) {
        array.clear();
        int half_window_size = floor(window_size / 2);
        for (int i = 0; i < index_array.size(); i++) {
            if (i - half_window_size >= 0 && i + half_window_size < index_array.size()) {
                double no_zero_elements = 0, average = 0;
                for (int j = i - half_window_size, k = 0; k < window_size; j++, k++) {
                    average += scan->ranges[index_array[j]];
                    if (scan->ranges[index_array[j]] > 0) {
                        no_zero_elements++;
                    }
                }

                array.push_back(no_zero_elements == 0 ? 0 : average / no_zero_elements);
            }
        }
    }

    bool isCollision(const vector<double> &average_array, int startIndex, float distance) {
        for (int i = 0 ; i < obstacle_size ; i++) {
            if (i + startIndex >= average_array.size()) {
                return false;
            }

            if (average_array[i + startIndex] > distance) {
                return false;
            }
        }

        return true;
    }

    int detectCollision(const vector<double> &average_array, float distance_stop, float distance_slow) {
        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_stop)) {
                return 0;
            }
        }

        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_slow)) {
                return 2;
            }
        }

        return 1;
    }

    int detectCollision(const vector<double> &average_array, float distance_stop) {
        for (int i = 0 ; i < average_array.size() ; i++) {
            if (isCollision(average_array, i, distance_stop)) {
                return 0;
            }
        }


        return 1;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<double> arrayList[6];
        for (int i = 0 ; i < 6 ; i++) {
            filter(arrayList[i], index_group[i], scan);
        }

        std_msgs::Int16MultiArray msg;
        msg.data={0,0,0,0,0,0};
        msg.data[top]=detectCollision(arrayList[top], top_distance_stop, top_distance_slow);
        msg.data[bottom]=detectCollision(arrayList[bottom], bottom_distance_stop, bottom_distance_slow);
        msg.data[top_L]=detectCollision(arrayList[top_L], top_corner_distance_stop);
        msg.data[top_R]=detectCollision(arrayList[top_R], top_corner_distance_stop);
        msg.data[bottom_L]=detectCollision(arrayList[bottom_L], bottom_corner_distance_stop);
        msg.data[bottom_R]=detectCollision(arrayList[bottom_R], bottom_corner_distance_stop);
        pub.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "avoidance");
    Group group;

    ros::spin();

    return 0;
}
