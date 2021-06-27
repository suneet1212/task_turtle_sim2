#include <cppad/ipopt/solve.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <sstream>
#include "turtlesim/Pose.h"

// obstacle position:
float obs_x;
float obs_y;

// target position:
double target_x = 9.5;
double target_y = 9.5;
double target_theta = -3.14;

// current position
float x_curr;
float y_curr;
float theta_curr;
float vel_curr;
float ang_vel_curr;
float dist = 20;

// Other constants of motion
double T = 0.1; // Sampling time period
int N = 10; // Horizon
int marker = 0;

// 
int done = 0;
typedef CPPAD_TESTVECTOR( double ) Dvector;
size_t nx = 2*(N);
Dvector X_initial(nx);
Dvector X_lower(nx);
Dvector X_upper(nx);

size_t ng = 2*N+4;
Dvector gl(ng), gu(ng);

namespace
{
    using CppAD::AD;
    class FG_eval
    {
        public:
            typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
            void operator()(ADvector& fg, const ADvector X)
            {    
                // const ADvector positions, const ADvector velocity, const ADvector obs_position
                // X_curr is the current position
                // assert(positions.size() == 3*(N+1)); // the position vectors have been flattened
                // assert(velocity.size() == 2*(N+1)); // so are the velocity vectors
                // assert(obs_position.size() == 2);
                // ROS_INFO("fg size = %d", fg.size());
                assert(fg.size() == (2*N)+5);
                assert(X.size() == 2*N);

                // Fortran style indexing
                // one for loop reqd, and optimisation is needed for X[N+1]th position to be closest to the target point
                // AD<double> x[N+1];
                // AD<double> y[N+1];
                // AD<double> theta[N+1];
                // AD<double> curr_x = X[0];
                // AD<double> curr_y = X[1];
                // AD<double> curr_theta = X[2];                
                AD<double> vel[N];
                AD<double> ang_vel[N];
                for (int i = 0; i < N; i++)
                {
                    // x[i] = X[5*i];
                    // y[i] = X[5*i + 1];
                    // theta[i] = X[5*i + 2];
                    vel[i] = X[2*i];
                    ang_vel[i] = X[2*i+1];
                }
                // AD<double> obs_x = X[5*(N+1)];
                // AD<double> obs_y = X[5*(N+1)+1];
                
                AD<double> x_n = x_curr;
                AD<double> y_n = y_curr;
                AD<double> theta_n = theta_curr; 
                for (int i = 0; i < N; i++)
                {
                    x_n += T*vel[i]*CppAD::cos(theta_n);
                    y_n += T*vel[i]*CppAD::sin(theta_n);
                    theta_n += T*ang_vel[i];
                }
                
                // f(x)
                // optimizing eqn:
                fg[0] = pow(x_n-target_x,2) + pow(y_n-target_y,2) + pow(theta_n-target_theta,2);
                // for (int i = 0; i < N; i++)
                // {
                //     // fg[0] += pow(x[i]-target_x,2) + pow(y[i]-target_y,2) + pow(theta[i]-target_theta,2);
                //     // fg[0] += pow(vel[i+1]-vel[i],2) + pow(ang_vel[i+1]-ang_vel[i],2);

                // }

                // Constraints:
                // N eqns of velocity and position relation:
                // for (int i = 0; i < N; i++)
                // {
                //     fg[3*i + 1] = x[i+1] - x[i] - T*vel[i]*CppAD::cos(theta[i]); // check if the angles for the cosine functions are in radians or degrees
                //     fg[3*i + 2] = y[i+1] - y[i] - T*vel[i]*CppAD::sin(theta[i]); // check if the angles for the cosine functions are in radians or degrees
                //     fg[3*i + 3] = theta[i+1] - theta[i] - T*ang_vel[i]; // check if the angles for the cosine functions are in radians or degrees
                // }
                // completed till fg[3N];
                // all the above 3N constraints are equality and equal to 0;

                // eqn for the distance from the obstacle is greater than 2 at each time step
                fg[1] = (x_n- obs_x)*(x_n- obs_x) + (y_n- obs_y)*(y_n- obs_y); // values greater than 4
                for (int i = 0; i < N-1; i++)
                {
                    fg[2*i+2] = abs(vel[i+1] - vel[i]);
                    fg[2*i+3] = abs(ang_vel[i+1] - ang_vel[i]);
                }
                fg[2*N+2] = x_n;
                fg[2*N+3] = y_n;
                fg[2*N+4] = theta_n;

                // completed till fg[4N];
                return;
            }
     };
}


void obs_position_callback(const turtlesim::Pose& msg)
{
    obs_x = msg.x;
    obs_y = msg.y;
}
void curr_position_callback(const turtlesim::Pose& msg)
{
    // take current position and then call the function to solve for the equations.
    // create global position and velocity vectors.
    // Initialise Position as a straight line from current pos to final pos and velocity as same as current velocity
    // ROS_INFO("dist1 = %ld", dist);
    x_curr = msg.x;
    y_curr = msg.y;
    theta_curr = msg.theta;
    vel_curr = msg.linear_velocity;
    ang_vel_curr = msg.angular_velocity;
    
    dist = pow(pow(x_curr-target_x,2) + pow(y_curr-target_y,2) + pow(theta_curr-target_theta,2),0.5);
    // dist = pow(x_curr-target_x,2) + pow(y_curr-target_y,2) + pow(theta_curr-target_theta,2);
    std::cout << "Current x-coord : " << x_curr << "\n";
    std::cout << "Current y-coord : " << y_curr << "\n";
    std::cout << "Current theta : " << theta_curr << "\n";
}
void target_position_callback(const turtlesim::Pose& msg)
{
    target_x = msg.x;
    target_y = msg.y;
    target_theta = msg.theta;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_task");
    ros::NodeHandle n;
    ros::Subscriber turt1_pose_sub = n.subscribe("/turtle1/pose", 10, obs_position_callback);
    ros::Subscriber turt2_pose_sub = n.subscribe("/turtle2/pose", 10, curr_position_callback);
    ros::Subscriber turt2_target_sub = n.subscribe("/target_pose", 10, target_position_callback);
    ros::Publisher turt2_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",10);
    
    geometry_msgs::Twist velocity;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    
    for (int i = 0; i < N; i++)
    {
        X_lower[2*i] = -1.5;
        X_lower[2*i+1] = -1.0;

        X_upper[2*i] = 1.5;
        X_upper[2*i+1] = 1.0;
    }
    gl[0] = 4.0;
    gu[0] = 1e19;
    for (int i = 1; i < 2*N+1; i++)
    {
        gl[i] = 0;
        gu[i] = 0.5;
    }
    gl[2*N+1] = 0; gu[2*N+1] = 10;
    gl[2*N+2] = 0; gu[2*N+2] = 10;
    gl[2*N+3] = -3.14; gu[2*N+3] = 3.14;
    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    // std::string options;
    // // turn off any printing
    // options += "Integer print_level  0\n";
    // options += "String  sb           yes\n";
    // // maximum number of iterations
    // // options += "Integer max_iter     100\n";
    // // approximate accuracy in first order necessary conditions;
    // // see Mathematical Programming, Volume 106, Number 1,
    // // Pages 25-57, Equation (6)
    // options += "Numeric tol          1e-6\n";
    // // derivative testing
    // options += "String  derivative_test            second-order\n";
    // // maximum amount of random pertubation; e.g.,
    // // when evaluation finite diff
    // options += "Numeric point_perturbation_radius  0.\n";
    
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // Disables printing IPOPT creator banner
    options += "String  sb          yes\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // ros::Rate loop_rate(T);
    int count = 0;
    // place to return solution
    // ROS_INFO("dist: %ld", dist);
    CppAD::ipopt::solve_result<Dvector> solution;
    while ((dist > 0.01) && (ros::ok()))
    {
        if(marker == 0)
        {
            for (int i = 0; i < N; i++)
            {
                X_initial[2*i] = vel_curr;
                X_initial[2*i + 1] = ang_vel_curr;
            }
            marker = 1;
        }
        else
        {
            for (int i = 0; i < N; i++)
            {
                X_initial[2*i] = solution.x[2*i];
                X_initial[2*i + 1] = solution.x[2*i + 1];
            }
        }
        // initial values have been declared.

        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, X_initial, X_lower, X_upper, gl, gu, fg_eval, solution
        );
        // publish
        // ROS_INFO("solutions_status: %d", solution.status);
        velocity.linear.x = solution.x[0];
        velocity.angular.z = solution.x[1];
        // std::cout << velocity << "\n";
        double begin = ros::Time::now().toSec();
        while (ros::Time::now().toSec() < begin+T)
        {
            turt2_vel_pub.publish(velocity);
            // std::cout << count << "\n";
        }
        ros::spinOnce();
        // count += 1;
        // loop_rate.sleep();

        // ros::Duration(T).sleep();
    }
    ros::spin();

  return 0;
}