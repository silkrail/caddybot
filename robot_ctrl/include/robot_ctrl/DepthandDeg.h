#ifndef ROBOT_CTRL_DEPTHANDDEG_H
#define ROBOT_CTRL_DEPTHANDDEG_H

#include <ros/ros.h>

namespace robot_ctrl {

struct DepthandDeg {
    float center_depth;
    float deg;

    DepthandDeg() : center_depth(0.0), deg(0.0) {}

    DepthandDeg(float center_depth, float deg) : center_depth(center_depth), deg(deg) {}

    void clear() {
        center_depth = 0.0;
        deg = 0.0;
    }

    bool operator==(const DepthandDeg &other) const {
        return center_depth == other.center_depth && deg == other.deg;
    }

    bool operator!=(const DepthandDeg &other) const {
        return !(*this == other);
    }

    void publish(ros::Publisher &pub) const {
        pub.publish(*this);
    }

    void advertise(ros::NodeHandle &nh, const std::string &topic, uint32_t queue_size) {
        pub = nh.advertise<DepthandDeg>(topic, queue_size);
    }

    void subscribe(ros::NodeHandle &nh, const std::string &topic, uint32_t queue_size,
                   void (*callback)(const DepthandDeg &)) {
        sub = nh.subscribe(topic, queue_size, callback);
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
};

} // namespace robot_ctrl

#endif // ROBOT_CTRL_DEPTHANDDEG_H

