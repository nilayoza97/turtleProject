/**
 * @file turtle_project_node.cpp
 * @author Nilay Oza (nilayoza97@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <boost/random.hpp>

/**
 * @brief Type of turtle : Police, Robber, Grid
 * 
 */
typedef enum turtleName
{
    RT, PT, GT
}turtle_name_t;

turtle_name_t turtle_name;

/**
 * @brief global poses to access poses easily.
 * this has to be re-adjusted, global poses are a bad design.
 * had to be done due to time constraints and 
 * issues with subsciber callbacks of the class
 * 
 */
turtlesim::Pose pt_pose_msg, rt_real_pose_msg, rt_noisy_pose_msg;

/**
 * @brief Class for controlling turtles using turtlesim
 * 
 */
class SimTurtle
{

public :

    /**
     * @brief pose messages of different turtles
     * 
     */
    turtlesim::Pose cur_pose_msg,
                    cir_pose_msg, 
                    cir_cmd, 
                    rt_pose_msg,
                    /*pt_pose_msg,*/
                    gt_pose_msg;

    /**
     * @brief Get the Turtle Pose object
     * 
     * @return turtlesim::Pose : pose message of the requested turtle
     */
    turtlesim::Pose getTurtlePose()
    {
        if (turtle_name == RT)
        {
            return this->rt_pose_msg;
        }
        else if(turtle_name == PT)
        {
            return pt_pose_msg;
        }
        else if (turtle_name == GT)
        {
            return this->gt_pose_msg;
        }
        else
        {
            ROS_INFO("Invalid Turtle");
        } 
    }
    /**
     * @brief euclidean distance from the current turtle location to the goal
     * 
     * @param goal_pose_msg pose of the goal location
     * @param cur_pose_msg current pose of the turtle
     * @return double distance value
     */
    double goalPoseDistance(turtlesim::Pose goal_pose_msg, turtlesim::Pose cur_pose_msg)
    {
        return sqrt(pow((goal_pose_msg.x - cur_pose_msg.x), 2) +
                            pow((goal_pose_msg.y - cur_pose_msg.y), 2));
    }

    /**
     * @brief angle to turn to move towards the goal from current location
     * 
     * @param goal_pose_msg pose of the goal location
     * @param cur_pose_msg current pose of the turtle
     * @return double angle value to turn towards the goal
     */
    double goalOrientation(turtlesim::Pose goal_pose_msg, turtlesim::Pose cur_pose_msg)
    {
        return atan2((goal_pose_msg.y - cur_pose_msg.y), 
                        (goal_pose_msg.x - cur_pose_msg.x));
    }  

    /**
     * @brief Spawn a turtle using turtlesim, uses spawn service of turtlesim package
     * 
     * @param nh NodeHandle to initialize the spawn client
     * @param random true if needed to spawn a turtle at a random location, otherwise false
     * @param req_x requested x coordinate of the turtle to be spawned
     * @param req_y requested y coordinate of the turtle to be spawned
     * @param req_th requested heading angle theta of the turtle to be spawned
     * @return std::string name of the spawned turtle
     */
    std::string turtleSpawn(ros::NodeHandle &nh, bool random, int req_x =5, int req_y=5, int req_th=0)
    {
        ros::ServiceClient spawn_cli = nh.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn spawn_srv;

        if (random)
        {
            srand (time(NULL));    
            spawn_srv.request.x = rand() % 11;
            spawn_srv.request.y = rand() % 11;
            spawn_srv.request.theta = rand() % 3;
        }
        else
        {
            spawn_srv.request.x = req_x;
            spawn_srv.request.y = req_y;
            spawn_srv.request.theta = req_th;
        }

        if (spawn_cli.call(spawn_srv))
        {
          ROS_INFO("Spawned turtle name %s ", spawn_srv.response.name.c_str());
        }
        else
        {
          ROS_ERROR("Failed to call service spawn");
          return std::string("failed spawn");
        }
        return spawn_srv.response.name;
    }

    /**
     * @brief rotates the turtle by publishing Twist message to the requested turtle command velocity publisher
     * 
     * @param angle the angle that needs to be rotated in radians
     * @param twist_pub requested turtle's command velocity publisher
     * @param vel_msg [in] twist velocity command message type
     * @param rotated [out] true if the turtle has completed the rotation, otherwise false
     * @param cur_pose_msg current pose of the turtle requesting rotation
     */
    void rotateTurtle(double angle, ros::Publisher &twist_pub, geometry_msgs::Twist &vel_msg, bool &rotated, turtlesim::Pose cur_pose_msg)
    {
        // actual angle to move = absolute angle to move - current theta
        double diff_angle = angle - cur_pose_msg.theta;
        ROS_INFO("Diff angle : %f", diff_angle);

        // Assumes rotation complete if the angle difference becomes within the bounds
        if(diff_angle > 0.05 || diff_angle < -0.05)
        {
            rotated = false;
            // slowed anglular velocity to not let it overshoot
            vel_msg.angular.z = 0.8 * (angle - cur_pose_msg.theta);
            twist_pub.publish(vel_msg);
        }
        else
        {
            rotated = true;
        }
    }

    /**
     * @brief Limit acceleration and deceleration of the turtle
     * 
     * @param vel_msg [out] sets the linear and angular acceleration and deceleration limits
     * @param last_vel_msg previous velocity command sent to the turtle
     * @param diff_time time difference between the current and the previous velocity command
     */
    void accelLimit(geometry_msgs::Twist &vel_msg, geometry_msgs::Twist last_vel_msg, ros::Duration &diff_time)
    {
        double diff_vel_lin = vel_msg.linear.x - last_vel_msg.linear.x;
        double diff_vel_ang = vel_msg.angular.z - last_vel_msg.angular.z;
        double dt = diff_time.toSec();
        dt /= 1e9;
        ROS_INFO("delta : %f", dt);

        // constraint linear velocity acceleration
        if(diff_vel_lin/dt > 3)
        {
             vel_msg.linear.x = 3 * dt;
        }
        // constraint angular velocity acceleration
        if(diff_vel_ang/dt > 0.6)
        {
            vel_msg.angular.z = 0.6 * dt;
        }
        // constraint linear velocity deceleration
        if(diff_vel_lin/dt < -3)
        {
            vel_msg.linear.x = -3 * dt;
        }
        // constraint linear velocity deceleration
        if(diff_vel_ang/dt < -0.6)
        {
            vel_msg.angular.z = -0.6 * dt;
        }
    }

    /**
     * @brief Move towards the goal that is requested by the caller
     * 
     * @param start_time time of starting the goal traversal to find the total traversal time
     * @param vel_msg [in] velocity command message to be sent to the turtle
     * @param last_vel_msg previous velocity command sent to the turtle
     * @param goal_pose_msg pose of the goal location
     * @param cur_pose_msg current pose of the turtle
     * @param reached_goal [out]true if the turtle has reached the goal given to it, otherwise false
     * @param max_vel maximum velocity at which the turtle can be allowed to move
     */
    void goalTraversal(ros::Time start_time, geometry_msgs::Twist &vel_msg, geometry_msgs::Twist last_vel_msg, 
                    turtlesim::Pose goal_pose_msg, turtlesim::Pose cur_pose_msg, bool &reached_goal, double max_vel =DBL_MAX)
    {

        //PID gains for linear and angular velocity
        float kp_linear = 1;
        float ki_linear = 0.000;
        float kd_linear = 0;
        float kp_angular = 3;
        float ki_angular = 0.000;
        float kd_angular = 0.2;

        float steady_state_error = 0.1;                     // acceptable limit when the turtle reaches the goal
        double linear_err=0, angular_err=0;                 // difference in goal and current pose
        double last_lin_err, last_ang_err;                  // error values calculated in the previous loop cycle
        double integral_lin_err, integral_ang_err;          // integral error terms
        double deriv_lin_err, deriv_ang_err;                // derivative error terms

        ros::Time end_time, curr_loop_time, last_time;
        ros::Duration travel_time;
        ros::Duration diff_time;

        linear_err = goalPoseDistance(goal_pose_msg, cur_pose_msg);
        angular_err = goalOrientation(goal_pose_msg, cur_pose_msg) - cur_pose_msg.theta;
        integral_lin_err += linear_err;
        integral_ang_err += angular_err; 
        deriv_lin_err = linear_err - last_lin_err;
        deriv_ang_err = angular_err - last_ang_err;
        if (goalPoseDistance(goal_pose_msg, cur_pose_msg) > steady_state_error)
        {
            reached_goal = false;
            curr_loop_time = ros::Time::now();
            diff_time = curr_loop_time - last_time;
            ROS_INFO("Diff time : %f", diff_time.toSec());

            vel_msg.linear.x = kp_linear * linear_err + ki_linear * integral_lin_err + kd_linear * deriv_lin_err;       //3 * 0.3;
            vel_msg.angular.z = kp_angular * angular_err + ki_angular * integral_ang_err + kd_angular * deriv_ang_err;       //1 * goalOrientation() - cur_pose_msg.theta;

            // constraint the maximum velocities of the turtle if requested
            if (vel_msg.linear.x > max_vel)
            {
                vel_msg.linear.x = max_vel;
            }
            if (vel_msg.angular.z > max_vel)
            {
                vel_msg.angular.z = max_vel;
            }

            ROS_INFO("VEL LIN : %f, VEL ANG : %f", vel_msg.linear.x, vel_msg.angular.z);

            // Limit acceleration and deceleration of the turtle
            accelLimit(vel_msg, last_vel_msg, diff_time);
            ROS_INFO(" ACC VEL LIN : %f, VEL ANG : %f", vel_msg.linear.x, vel_msg.angular.z);
            last_time = curr_loop_time;

            last_vel_msg.linear.x = vel_msg.linear.x;
            last_vel_msg.angular.z = vel_msg.angular.z;
            last_lin_err = linear_err;
            last_ang_err = angular_err;
        }
        else
        {
           reached_goal = true;
           end_time = ros::Time::now();
           travel_time = end_time - start_time;
           ROS_INFO("travel time : %f ", travel_time.toSec());
           // stop motion of the turtle
           vel_msg.linear.x = 0.0;
           vel_msg.angular.z = 0.0;
        }
    }

    /**
     * @brief Moves the turtle in a grid fashion starting from left bottom corner
     * 
     * @param gt_twist_pub grid turtle's command velocity publisher
     * @param nav_end [out] true if the entire grid is navigated, otherwise false
     */
    void grid_navigation(ros::Publisher &gt_twist_pub, bool &nav_end)
    {
        turtlesim::Pose rotate_pose_msg;
        double angle;                           // angle value to rotate before goal traversal
        static int loop_count = 0;
    
        std::vector<std::vector<int>>grid_points = {{1, 1}, {10, 1}, {10,2}, {1, 2}, {1, 3}, 
                                                {10, 3}, {10, 4}, {1, 4}, {1, 5}, {10, 5}, 
                                                {10, 6}, {1, 6}, {1, 7}, {10, 7}, {10, 8},
                                                {1, 8}, {1, 9}, {10, 9}, {10, 10}, {1, 10}};

        rotate_pose_msg.x = grid_points[loop_count].at(0);
        rotate_pose_msg.y = grid_points[loop_count].at(1);

        // set the angle values by iterating over the grid points
        if (loop_count == 0)
        {
            angle = goalOrientation(rotate_pose_msg, cur_pose_msg);
        }
        else if ((grid_points[loop_count].at(0) == 10 && (grid_points[loop_count].at(1) % 2) == 0)
                    || (grid_points[loop_count].at(0) == 1 && (grid_points[loop_count].at(1) % 2) == 1))
        {
            angle = 1.57;
        }
        else if (grid_points[loop_count].at(0) == 10 && (grid_points[loop_count].at(1) % 2) == 1)
        {
            angle = 0;
        }
        else if (grid_points[loop_count].at(0) == 1 && (grid_points[loop_count].at(1) % 2) == 0)
        {
            angle = 3.18;
        }
        else
        {
            angle = 0;
        }
        ROS_INFO("Angle : %f", angle);
        ROS_INFO("Loop Count : %i",loop_count);
        
        geometry_msgs::Twist vel_msg, last_vel_msg;
        turtlesim::Pose grid_pose_msg;
        turtle_name = GT;
        grid_pose_msg = this->getTurtlePose();
        ros::Time start_time = ros::Time::now();

        // check if the turtle rotation complete or goal reached
        static bool rotated = false, reached_goal = false;
        if(rotated == false)
        {
            ROS_INFO("Rotated False");
            rotateTurtle(angle, gt_twist_pub, vel_msg, rotated, grid_pose_msg);
        }
        if(rotated == true && reached_goal == false)
        {
            ROS_INFO("Rotated");
            goalTraversal(start_time, vel_msg, last_vel_msg, rotate_pose_msg, grid_pose_msg, reached_goal);
            gt_twist_pub.publish(vel_msg);
        }
        if(reached_goal == true)
        {
            ROS_INFO("Reached Goal");
            reached_goal = false;
            rotated = false;
            loop_count++;
        }

        // after all grid points are completed, end navigation
        if(loop_count > 19)
        {
            nav_end = true;
            ROS_INFO("Loop Count : %i, Nav End : %d", loop_count, nav_end);
        }
    }

    /**
     * @brief Pose callback of the turtle that moves in cirles
     * 
     * @param msg subscribed message data
     */
    void circTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        cir_pose_msg.x = msg->x;
        cir_pose_msg.y = msg->y;
        cir_pose_msg.theta = msg->theta;
        ROS_INFO("Circ Pose x : %f , y : %f , theta : %f \n",cir_pose_msg.x, cir_pose_msg.y, cir_pose_msg.theta);
    }

    /**
     * @brief moves the turtle in circles by publishing the velocity commands
     * 
     * @param velocity requested velocity with which the turtle must be moved in circles
     * @param radius requested radius of the circle in which the turtle moves
     * @param cir_twist_pub circle movement turtle's command velocity publisher
     */
    void circleTurtle(double velocity, double radius,ros::Publisher &cir_twist_pub)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = velocity/radius;
        rt_pose_msg.linear_velocity = cmd_vel.linear.x;
        rt_pose_msg.angular_velocity = cmd_vel.angular.z;
        cir_twist_pub.publish(cmd_vel);
    }

    /**
     * @brief Pose callback of the Robber Turtle
     * 
     * @param msg subscribed message data
     */
    void rtTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        this->rt_pose_msg.x = msg->x;
        this->rt_pose_msg.y = msg->y;
        this->rt_pose_msg.theta = msg->theta;
        ROS_INFO("Robber Turtle Pose x : %f , y : %f , theta : %f \n",rt_pose_msg.x, rt_pose_msg.y, rt_pose_msg.theta);
        // cur_pose_msg.x = msg->x;
        // cur_pose_msg.y = msg->y;
        // cur_pose_msg.theta = msg->theta;
        // ROS_INFO("Current Pose x : %f , y : %f , theta : %f \n",cur_pose_msg.x, cur_pose_msg.y, cur_pose_msg.theta);
    }

    /**
     * @brief Pose callback of the grid turtle
     * 
     * @param msg subscribed message data
     */
    void gtTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        this->gt_pose_msg.x = msg->x;
        this->gt_pose_msg.y = msg->y;
        this->gt_pose_msg.theta = msg->theta;
        ROS_INFO("Grid Turtle Pose x : %f , y : %f , theta : %f \n",gt_pose_msg.x, gt_pose_msg.y, gt_pose_msg.theta);
    }


    /**
     * @brief timer callback of the robber turtle to publish its commands
     * 
     * @param circular_pub velocity command publisher of the robber turtle that moves in circles
     * @param cir_gaus_pub velocity command publisher of the robber turtle with added gaussian noise to its pose
     */
    void timerCallback(const ros::TimerEvent&, ros::Publisher circular_pub, ros::Publisher cir_gaus_pub)
    {
        turtlesim::Pose robber_pose, noisy_pose_msg;
        turtle_name = RT;
        robber_pose = getTurtlePose();
        noisy_pose_msg = robber_pose;
        boost::random::mt19937 rng(std::time(0));
        double mean, std_dev = 10;
        boost::random::normal_distribution<> dist(mean, std_dev);
        noisy_pose_msg.x += dist(rng);
        noisy_pose_msg.y += dist(rng);
        noisy_pose_msg.theta += dist(rng);
        noisy_pose_msg.linear_velocity += dist(rng);
        noisy_pose_msg.angular_velocity += dist(rng);
        circular_pub.publish(robber_pose);
        cir_gaus_pub.publish(noisy_pose_msg);
    }

    /**
     * @brief Callback for Police turtle to access Real pose of the Robber turtle
     * 
     * @param msg subscribed message data
     */
    void rtRealTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        rt_real_pose_msg.x = msg->x;
        rt_real_pose_msg.y = msg->y;
        rt_real_pose_msg.theta = msg->theta;
    }

    /**
     * @brief Callback of the Police turtle to get its pose
     * 
     * @param msg subscribed message data
     */
    void ptTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        pt_pose_msg.x = msg->x;
        pt_pose_msg.y = msg->y;
        pt_pose_msg.theta = msg->theta;

        ROS_INFO("PT POSE RECEIVED x : %f, y: %f", pt_pose_msg.x, pt_pose_msg.y);
    }

    /**
     * @brief Callback for Police turtle to access noisy pose of the Robber turtle
     * 
     * @param msg subscribed message data
     */
    void ptNoisyTurtlePoseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        rt_noisy_pose_msg.x = msg->x;
        rt_noisy_pose_msg.y = msg->y;
        rt_noisy_pose_msg.theta = msg->theta;
    }

    // SimTurtle()
    // {
    //     //cir_twist_pub = nh.advertise<geometry_msgs::Twist>(sp_srv.response.name + "/cmd_vel",1);
    //  }

private:

};

int main(int argc, char **argv)
{
    /* Initialize the turtle project node */
    ros::init(argc, argv, "turtle_project_node");
    ros::NodeHandle nh;
    
    /* Publishers of all the turtles */
    ros::Publisher rt_twist_pub, pt_twist_pub, gt_twist_pub, circular_pub, cir_gaus_pub;
    
    /* Names of the Robber, Police and Grid turtles */
    std::string rt, pt, gt;

    //cir_twist_pub = nh.advertise<geometry_msgs::Twist>(sp_srv.response.name + "/cmd_vel",1);
    //ros::Subscriber circ_pose_sub = nh.subscribe(sp_srv.response.name + "/pose", 1000, circTurtlePoseCallback);

    ros::Rate loop_rate(10);
    
    /* Command velocity messages */
    geometry_msgs::Twist vel_msg, last_vel_msg;

    SimTurtle sim_turtle;

    /* Test goal pose for the goal Traversal function*/
    turtlesim::Pose g_pose_msg;
    turtle_name = PT;
    g_pose_msg.x = 1;
    g_pose_msg.y = 1;

    ros::Time start_time = ros::Time::now();
    bool nav_end = false;                               // true if grid navigation completes
    bool reached_goal_m;                                // true if the goal traversal reaches the goal
    bool rand = true;                                   // random spawn of the turtle is required
    bool spawned_pt = false;                            // Police turtle spawned is initialized as false
    double turtle_vel = 2.0;                            // velocity to move the turtle in circles
    double max_vel = turtle_vel/2.0;                    // maximum velocity the turtle is allowed to move
    int pt_time;

    gt = sim_turtle.turtleSpawn(nh, true);
    rt = sim_turtle.turtleSpawn(nh, false, 6, 2, 0);
    gt_twist_pub = nh.advertise<geometry_msgs::Twist>(gt + "/cmd_vel",1);
    ros::Subscriber pose_sub = nh.subscribe(rt + "/pose", 1000, &SimTurtle::rtTurtlePoseCallback, &sim_turtle);
    rt_twist_pub = nh.advertise<geometry_msgs::Twist>(rt + "/cmd_vel",1);
    circular_pub = nh.advertise<turtlesim::Pose>("/rt_real_pose",1);
    cir_gaus_pub = nh.advertise<turtlesim::Pose>("/rt_noisy_pose",1);
    // ros::Subscriber circ_pose_sub = nh.subscribe(sp_srv.response.name + "/pose", 1000, circTurtlePoseCallback);
    ros::Subscriber gt_pose_sub = nh.subscribe(gt + "/pose", 1000, &SimTurtle::gtTurtlePoseCallback, &sim_turtle);
    ros::Timer timer = nh.createTimer(ros::Duration(5.0), boost::bind(&SimTurtle::timerCallback, &sim_turtle, _1, boost::ref(circular_pub), boost::ref(cir_gaus_pub)));
    ros::Subscriber rt_pose_sub = nh.subscribe("/rt_real_pose", 1000, &SimTurtle::rtRealTurtlePoseCallback, &sim_turtle);
    ros::Subscriber rt_noisy_pose_sub = nh.subscribe("/rt_noisy_pose", 1000, &SimTurtle::ptNoisyTurtlePoseCallback, &sim_turtle);

    pt = sim_turtle.turtleSpawn(nh, rand);
    ros::Subscriber pt_pose_sub = nh.subscribe(pt + "/pose", 1000, &SimTurtle::ptTurtlePoseCallback, &sim_turtle);

    while(ros::ok())
    {
        pt_time = ros::Time::now().toSec() - start_time.toSec();
        if(pt_time == 10 && spawned_pt == false)
        {   
            pt_twist_pub = nh.advertise<geometry_msgs::Twist>(pt + "/cmd_vel",1);
            ROS_INFO("Spawned PT");
            spawned_pt = true;
        }
        if(nav_end == false)
        {
            //sim_turtle.grid_navigation(gt_twist_pub, nav_end);
        }
        sim_turtle.circleTurtle(turtle_vel, 4, rt_twist_pub);
        sim_turtle.goalTraversal(start_time, vel_msg, last_vel_msg, rt_real_pose_msg, pt_pose_msg, reached_goal_m, max_vel);
        if (sim_turtle.goalPoseDistance(rt_real_pose_msg, pt_pose_msg) < 3)
        {
            ROS_INFO(" Police Caught the Robber");
        }
        //sim_turtle.goalTraversal(start_time, vel_msg, last_vel_msg, rt_noisy_pose_msg, pt_pose_msg, reached_goal_m, max_vel);
        if(spawned_pt == true)
        {
            pt_twist_pub.publish(vel_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
