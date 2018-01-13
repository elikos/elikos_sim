/**
 * \file sim_roombas.cpp
 * \brief Simulation manager for roomba_sim. Creates and handles robots and quad.
 * \author christophebedard
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include "elikos_roomba/GroundRobot.h"
#include "elikos_roomba/ObstacleRobot.h"
#include "Quad.h"

static const double LOOP_RATE_SIM = 10.0; /**< loop rate */
static const std::string NODE_NAME = "sim_roombas"; /**< node name */

static const double TARGET_ROBOT_RADIUS = 1.0; /**< circle radius for target robot initial position */
static const double OBSTACLE_ROBOT_RADIUS = 5.0; /**<  circle radius for obstacle robot initial position */

double arenaDimension;

bool isTargetRobotOutsideArena(tf::Vector3 robotPosition) {
    return !((robotPosition.getX() >= (-arenaDimension/2.0)) && (robotPosition.getX() <= (arenaDimension/2.0))
             && (robotPosition.getY() >= (-arenaDimension/2.0)) && (robotPosition.getY() <= (arenaDimension/2.0)));
}

bool isTargetRobotAcrossGreenLine(tf::Vector3 robotPosition) {
    return (robotPosition.getY() > (arenaDimension/2.0));
}

bool isTargetRobotAcrossRedLine(tf::Vector3 robotPosition) {
    return (robotPosition.getY() < (-arenaDimension/2.0));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::Rate rate(LOOP_RATE_SIM);

    ros::NodeHandle n_p("~");
    int nbTargetRobots, nbObstacleRobots;
    n_p.getParam("/target_robot_count", nbTargetRobots);
    n_p.getParam("/obstacle_robot_count", nbObstacleRobots);
    n_p.getParam("arena_dimension", arenaDimension);

    std::list<std::unique_ptr<Robot>> robots;

    // create target robots
    for (int i = 0; i < nbTargetRobots; ++i) {
        double pos_x = TARGET_ROBOT_RADIUS * cos(i * (360 / nbTargetRobots) * DEG_TO_RAD);
        double pos_y = TARGET_ROBOT_RADIUS * sin(i * (360 / nbTargetRobots) * DEG_TO_RAD);
        double yaw = i * (360 / nbTargetRobots) * DEG_TO_RAD;
        std::string color = (i%2 == 0) ? "red" : "green";
        std::unique_ptr<Robot> uptr(new GroundRobot(n, i+1, tf::Vector3(pos_x, pos_y, 0.0), yaw, color));
        robots.push_back(std::move(uptr));
    }

    // create obstacle robots
    std::vector<std::string> heights = {"05", "10", "15", "20"};
    for (int i = 0; i < nbObstacleRobots; ++i) {
        double pos_x = OBSTACLE_ROBOT_RADIUS * cos(i * (360 / nbObstacleRobots) * DEG_TO_RAD);
        double pos_y = OBSTACLE_ROBOT_RADIUS * sin(i * (360 / nbObstacleRobots) * DEG_TO_RAD);
        double yaw = (-90.0 + (i * (360 / nbObstacleRobots))) * DEG_TO_RAD;
        std::string height = heights[i%4];
        std::unique_ptr<Robot> uptr(new ObstacleRobot(n, i+1, tf::Vector3(pos_x, pos_y, 0.0), yaw, height));
        robots.push_back(std::move(uptr));
    }

    // create quad
    std::unique_ptr<Quad> quad(new Quad(n, rate.expectedCycleTime()));

    // init sim
    int nbTargetRobotsAcrossGreenLine = 0;

    // run simulation
    while (ros::ok())
    {
        // quad
        quad->update();

        // robots
        for (auto robotBaseIt = robots.begin(); robotBaseIt != robots.end(); ++robotBaseIt) {
            (*robotBaseIt)->update();

            // if robot is active
            if ((*robotBaseIt)->isActive()) {
                // if target robot
                if ((*robotBaseIt)->getRobotType() == "ground") {
                    // check robot bumper collisions
                    for (auto robotIt = robots.begin(); robotIt != robots.end(); ++robotIt) {
                        // if not same robot
                        if (!(*(*robotIt) == *(*robotBaseIt))) {
                            (*robotBaseIt)->checkRobotCollision((*robotIt)->getPosition());
                        }
                    }

                    // check top interaction
                    (*robotBaseIt)->checkTopInteraction(quad->getPosition(), quad->getInteractionDiameter());

                    // check if robot crossed a line
                    if (isTargetRobotOutsideArena((*robotBaseIt)->getPosition())) {
                        // check which line was crossed
                        if (isTargetRobotAcrossGreenLine((*robotBaseIt)->getPosition())) {
                            ++nbTargetRobotsAcrossGreenLine;
                            ROS_INFO_STREAM((*robotBaseIt)->getNamespace() + " crossed green line (total = " + std::to_string(nbTargetRobotsAcrossGreenLine) + ")");
                        } else if (isTargetRobotAcrossRedLine((*robotBaseIt)->getPosition())) {
                            ROS_INFO_STREAM((*robotBaseIt)->getNamespace() + " crossed red line");
                        } else {
                            ROS_INFO_STREAM((*robotBaseIt)->getNamespace() + " crossed white line");
                        }

                        // deactivate robot
                        (*robotBaseIt)->deactivateRobot();
                    }
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}