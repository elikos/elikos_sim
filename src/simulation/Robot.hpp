#ifndef SIM_ROBOT_HPP
#define SIM_ROBOT_HPP

#include <ros/ros.h>

#include <tf/tf.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace elikos_sim {

/**
* Abstract base class representing robots in the arena (obstacles and targets)
*/
class Robot{
public:
    std::string getName() const { return this->name; }
    int getID() { return this->id; }
    tf::Transform getTransform() const { return this->transform; }
    virtual visualization_msgs::Marker getVizMarker() = 0;

    virtual void collide() = 0;
    virtual void move(ros::Duration cycleTime) = 0;

protected:
    Robot(int id, double simSpeed);
    void refreshTransform();

    int id;
    std::string name;
    double x, y, z, yaw, turnAngle, simSpeed;
    tf::Transform transform;
    tf::Vector3 v;
    tf::Quaternion q;
private:

    /* *************************************************************************************************
     * ***           HIDDEN CONSTRUCTORS (do not implement)
     * *************************************************************************************************
     */

    Robot();
    Robot& operator= (const Robot&);
    Robot (const Robot&);
};

} // namespace elikos_sim

#endif // SIM_ROBOT_HPP
