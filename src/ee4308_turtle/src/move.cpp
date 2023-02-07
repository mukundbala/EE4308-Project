#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <functional>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"
#define INF 100000

bool target_changed = false;
Position target_position(INF,INF);
Position robot_position(0, 0);
bool received_target_position = false;
double robot_angle = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_position.x = msg->point.x;
    target_position.y = msg->point.y;
    received_target_position = true;
    if (received_target_position)
    {
        ROS_INFO("TMOVE: Received Target");
    }
    ROS_INFO_STREAM("TMOVE: Target Coordinates Received: (" << target_position.x << "," << target_position.y<<")");
}

void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    robot_position.x = p.x;
    robot_position.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    robot_angle = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move;
    bool verbose;
    
    double Kp_lin;
    double Ki_lin;
    double Kd_lin;
    double max_lin_vel;
    double max_lin_acc;
    
    double Kp_ang;
    double Ki_ang;
    double Kd_ang;
    double max_ang_vel;
    double max_ang_acc;
    
    double move_iter_rate;

    bool tune_mode;
    bool tune_lin;
    bool tune_ang;

    std::string coupling_function_name;
    std::function<double(double)>coupling_func;

    if (!nh.param("enable_move", enable_move, true))
    {
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    }
        
    if (!nh.param("verbose_move", verbose, false))
    {
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    }

    if (!nh.param("Kp_lin", Kp_lin, 1.0))
    {
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    }
        
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
    {
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    }
        
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
    {
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    }
        
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
    {
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    }
        
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
    {
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    }
        
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
    {
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    }
        
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
    {
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    }
        
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
    {
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    }
        
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
    {
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    }
        
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
    {
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    }
        
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
    {
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");
    }

    if (!nh.param("tune_mode" , tune_mode , false))
    {
        ROS_WARN(" TMOVE: Param tune_mode not found, set to false");
    }

    if (!nh.param("tune_lin" , tune_lin , false))
    {
        ROS_WARN("TMOVE: Param tune_lin not found, set to false");
    }

    if (!nh.param("tune_ang" , tune_ang , false))
    {
        ROS_WARN(" TMOVE: Param tune_ang not found, set to false");
    }

    if (!nh.param("coupling_func" , coupling_function_name , std::string("cos")))
    {
        ROS_WARN(" TMOVE: Param tune_ang not found, set to false");
    }

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    //setup coupling function
    if (coupling_function_name == "quadratic")
    {
        coupling_func = dampingQuadratic;
    }
    
    else if (coupling_function_name == "cos")
    {
        coupling_func = dampingCos;
    }

    else
    {
        coupling_func = dampingCos;
    }

    if (!tune_mode) //just a safeguard in case we forget (see turtle.yaml)
    {
        tune_lin = false;
        tune_ang = false; 
    }
    else
    {
        if ((!tune_lin && !tune_ang) || (tune_ang && tune_lin))
        {
            ROS_INFO("Both tune_ang and tune_lin cannot have the same boolean value. Setting tune_mode to false!");
            tune_mode = false;
        }
    }
    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    // we keep pinging the topic until we have received robot pos and target pos.
    while (ros::ok() && nh.param("run", true) && robot_angle == 10 && robot_position.x == INF)
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0;
    double cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    double prev_linear_error = 0.0;
    double cumulative_linear_error = 0.0;
    double prev_angular_error = 0.0;
    double cumulative_angular_error = 0.0;

    ////////////////// DECLARE VARIABLES HERE //////////////////

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0 || robot_angle == 10 || target_position.x == INF) // ros doesn't tick the time fast enough.
            {
                continue;
            }
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            if (tune_mode)
            {
                ROS_INFO("[Move:] In Tune Mode");
                ROS_INFO_STREAM("[Move:] : Target Pos: (" << target_position.x << "," << target_position.y<<")");
                ROS_INFO_STREAM("[Move:] : Robot Pose: (" << robot_position.x << "," << robot_position.y<<","<<robot_angle <<")");
                double curr_linear_error = target_position.x - robot_position.x; //the current euclidean error between robot_pos and target_pos only in the x dir for tuning
                cumulative_linear_error += (curr_linear_error * dt); //update cumulative error
                double p_cmd_lin = curr_linear_error * Kp_lin; //proportional gain
                double i_cmd_lin = cumulative_linear_error * Ki_lin; //integral gain
                double d_cmd_lin = (curr_linear_error - prev_linear_error) / dt; //derivative gain
                cmd_lin_vel = p_cmd_lin + i_cmd_lin + d_cmd_lin;

                ROS_INFO_STREAM("Goal heading: " <<atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) << " Robot Heading: " << robot_angle);
                double curr_angular_error = atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_angle; //angular error
                cumulative_angular_error += (curr_angular_error * dt); //update cumulative error
                double p_cmd_ang = curr_angular_error * Kp_ang; //proportional gail
                double i_cmd_ang = cumulative_angular_error * Ki_ang; //integral gain
                double d_cmd_ang = (curr_angular_error - prev_angular_error) / dt; //derivative gain
                cmd_ang_vel = p_cmd_ang + i_cmd_ang + d_cmd_ang;
                ROS_INFO_STREAM("[Move:] : Linear Error:" << curr_linear_error << " " << "Angular Error: " << curr_angular_error);
                if (tune_lin)
                {
                    cmd_ang_vel = 0; //we only tune linear velocity here
                }

                if (tune_ang)
                {
                    cmd_lin_vel = 0; //we only tune angular velocity here
                }
                ROS_INFO_STREAM("[Move:] Cmd Lin Vel:" << cmd_lin_vel);
                ROS_INFO_STREAM("[Move:] Cmd Ang Vel:" << cmd_ang_vel);
                //update previous store
                prev_linear_error = curr_linear_error;
                prev_angular_error = curr_angular_error;
            }
            //coupling --> velocity damping from coupling --> constrain linear vel --> constrain angular vel
            else
            {
                double curr_linear_error = dist_euc(robot_position , target_position); //compute euclidean distance between robot and target pos
                cumulative_linear_error += (curr_linear_error * dt); //update cumulative linear error
                double p_cmd_lin = curr_linear_error * Kp_lin; //lin_vel proprotional gain
                double i_cmd_lin = cumulative_linear_error * Ki_lin; //lin_vel integral gain
                double d_cmd_lin = (curr_linear_error - prev_linear_error) / dt; //lin_vel derivative gain
                double raw_cmd_lin = p_cmd_lin + i_cmd_lin + d_cmd_lin; //the raw cmd_lin before any post processing

                double curr_angular_error = atan2(target_position.y - robot_position.y , target_position.x - robot_position.x); //the current angular error
                if (fabs(curr_angular_error) > M_PI / 2) //this ensures that all curr_angular error is strictly between +pi/2 and -pi/2
                {
                    raw_cmd_lin *= -1; //we are going to reverse to hit the target that is behind the robot aka in the 3rd and 4th quadrant of robot frame
                    curr_angular_error -= (sign(curr_angular_error) * M_PI); //forces angular error to be between [-pi/2 , pi/2]
                }
                cumulative_angular_error += (curr_angular_error * dt); //update cumulative angular error
                double p_cmd_ang = curr_angular_error * Kp_ang; //ang_cmd proportional gain
                double i_cmd_ang = cumulative_angular_error * Ki_lin; //ang_cmd integral gain
                double d_cmd_ang = (curr_angular_error - prev_angular_error) / dt; //ang_cmd derivative gain
                double raw_cmd_ang = p_cmd_ang + i_cmd_ang + d_cmd_ang; //the raw cmd_ang signal
                
                double damping_coeff = coupling_func(curr_angular_error); //use the coupling function initialised from before
                raw_cmd_lin *= damping_coeff; //coupling done. Can be positive or negative

                double curr_lin_acc = (raw_cmd_lin - cmd_lin_vel) / dt; //this will account for the direction of velocity. A negative lin_acc can mean "deceleration" or acceleration in th opp direction
                double sat_lin_acc = fabs(curr_lin_acc) > max_lin_acc ? sign(curr_lin_acc) * max_lin_acc : curr_lin_acc; //saturate acceleration
                double sat_lin_vel = fabs(cmd_lin_vel + sat_lin_acc * dt) > max_lin_vel ? sign(raw_cmd_lin) * max_lin_vel : cmd_lin_vel + sat_lin_acc * dt;
                
                double curr_ang_acc = (raw_cmd_ang - cmd_ang_vel) / dt;
                double sat_ang_acc = fabs(curr_ang_acc) > max_ang_acc ? sign(curr_ang_acc) * max_ang_acc : curr_ang_acc;
                double sat_ang_vel = fabs(cmd_ang_vel + sat_ang_acc * dt) > max_ang_vel ? sign(raw_cmd_ang) * max_ang_vel : cmd_ang_vel + sat_ang_acc * dt;

                //update all values for publishing and prep for next iteration
                cmd_lin_vel = sat_lin_vel;
                cmd_ang_vel = sat_ang_vel;
                prev_linear_error = curr_linear_error;
                prev_angular_error = curr_angular_error;
            }
            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}