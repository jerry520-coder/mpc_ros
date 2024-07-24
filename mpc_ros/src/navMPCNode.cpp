/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <iostream>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

#include "navMpc.h"
#include <Eigen/Core>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        int get_thread_numbers(); // 获取线程数量的方法
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl; // 订阅器，用于订阅里程计、路径、目标和AMCL（自适应蒙特卡洛定位）话题
        ros::Publisher _pub_globalpath,_pub_odompath, _pub_twist, _pub_mpctraj;   // 发布器，用于发布全局路径、里程计路径、速度命令和MPC轨迹
        ros::Timer _timer1;   // 定时器，用于控制控制循环的频率
        tf::TransformListener _tf_listener;
        ros::Time tracking_stime;  // 跟踪开始和结束时间
        ros::Time tracking_etime;
        ros::Time tracking_time;
        int tracking_time_sec;
        int tracking_time_nsec;
        

        std::ofstream file;
        unsigned int idx = 0;

        //time flag
        bool start_timef = false;  // 时间标记，用于记录控制循环的开始和结束时间
        bool end_timef = false;

        geometry_msgs::Point _goal_pos; // 目标位置
        nav_msgs::Odometry _odom;  // 里程计信息
        nav_msgs::Path _odom_path, _mpc_traj;   // 里程计路径和MPC轨迹
        //ackermann_msgs::AckermannDriveStamped _ackermann_msg;
        geometry_msgs::Twist _twist_msg;  // 速度命令消息

        // 全局路径、目标话题和坐标帧的名称
        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc; // MPC控制器
        map<string, double> _mpc_params; // MPC参数，以map存储

        // MPC步长和其他控制参数
        double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
               _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

        //double _Lf; 
        double _dt, _w, _throttle, _speed, _max_speed; // 车辆速度和控制参数
        double _pathLength, _goalRadius, _waypointsDist; // 路径长度、目标半径和路径点距离
        int _controller_freq, _downSampling, _thread_numbers; // 控制器频率、下采样和线程数量
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;  // 标志位，用于控制逻辑
        double polyeval(Eigen::VectorXd coeffs, double x); // 多项式评估函数
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order); // 多项式拟合函数

        // 回调函数，用于处理订阅的消息
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Parameters for control loop  // 控制循环的参数
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 0.50); // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 8.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    //pn.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver // MPC求解器的参数
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 1.0);
    pn.param("mpc_ref_etheta", _ref_etheta, 0.0);
    pn.param("mpc_w_cte", _w_cte, 5000.0);
    pn.param("mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("mpc_w_vel", _w_vel, 1.0);
    pn.param("mpc_w_angvel", _w_angvel, 100.0);
    pn.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 10.0);
    pn.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)// 最大角速度（弧度）
    pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel // 最大油门加速度
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables // 其他变量的界限值

    //Parameter for topics & Frame name // 主题和帧名的参数
    pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint" );

    //Display the parameters  // 显示参数
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: "  << _pub_twist_flag << endl;
    cout << "debug_info: "  << _debug_info << endl;
    cout << "delay_mode: "  << _delay_mode << endl;
    //cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "frequency: "   << _dt << endl;
    cout << "mpc_steps: "   << _mpc_steps << endl;
    cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "mpc_w_cte: "   << _w_cte << endl;
    cout << "mpc_w_etheta: "  << _w_etheta << endl;
    cout << "mpc_max_angvel: "  << _max_angvel << endl;

    //Publishers and Subscribers  // 发布者和订阅者
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    _sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);
    _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this);
    _sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCNode::amclCB, this);
    _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/global_path", 1); // Global path generated from another source  // 从其他来源生成的全局路径
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference  // MPC参考路径
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output // MPC输出轨迹
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)  // 对stage仿真器发布速度命令（不支持Ackermann消息）
    
    //Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc

    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _w = 0.0;
    _speed = 0.0;

    //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();



    idx = 0;
    file.open("/home/nscl1016/catkin_ws/src/mpc_ros/mpc.csv");
    file << "idx"<< "," << "cte" << "," <<  "etheta" << "," << "cmd_vel.linear.x" << "," << "cmd_vel.angular.z" << "\n";


    //Init parameters for MPC object    // 初始化MPC对象的参数
    _mpc_params["DT"] = _dt;
    //_mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_etheta;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_ANGVEL"]  = _w_angvel;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["ANGVEL"]   = _max_angvel;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params);
}

// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}

// Evaluate a polynomial.// 多项式求值函数，根据给定的系数和自变量x计算多项式的值
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    // 遍历多项式的系数
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);// 根据多项式的公式计算每一项的值并累加到结果中
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
// 多项式拟合函数，根据给定的x和y值拟合一个多项式
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    // 确保x和y的值的数量相同，并且阶数order在合理的范围内
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

     // 创建设计矩阵A，其大小为(xvals.size(), order + 1)
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) // 初始化矩阵的第一列为1
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)  // 填充设计矩阵的其余列
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }
    auto Q = A.householderQr(); // 使用Householder QR分解求解多项式的系数
    auto result = Q.solve(yvals); // 解算多项式的系数
    return result;
}

// CallBack: Update odometry // 里程计信息的回调函数，更新节点的里程计信息
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg; // 将传入的里程计信息赋值给节点的里程计成员变量
}

// CallBack: Update path waypoints (conversion to odom frame) // 路径点的回调函数，将路径点转换到里程计坐标系中
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // 如果目标已接收但未到达
    if(_goal_received && !_goal_reached)
    {    
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;// 初始化采样数量

            //find waypoints distance // 计算路径点之间的距离
            if(_waypointsDist <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy); // 计算欧几里得距离
                _downSampling = int(_pathLength/10.0/_waypointsDist);// 根据路径长度和距离计算下采样数量
            }            

            // Cut and downsampling the path // 截取并下采样路径
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;

                    // 将路径点从地图坐标系转换到里程计坐标系
                    _tf_listener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; // 更新总长度
                sampling = sampling + 1;  // 更新采样计数
            }
           
            // 如果转换后的路径点数量足够
            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame // 将转换后的路径点赋值给节点的路径成员变量
                _path_computed = true;// 设置路径已计算标志为真
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            else
            {
                cout << "Failed to path generation" << endl;
                _waypointsDist = -1;
            }
            //DEBUG            
            //cout << endl << "N: " << odom_path.poses.size() << endl 
            //<<  "Car path[0]: " << odom_path.poses[0];
            // << ", path[N]: " << _odom_path.poses[_odom_path.poses.size()-1] << endl;
        }
        catch(tf::TransformException &ex)// 捕获坐标变换异常
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

// CallBack: Update goal status // 目标状态更新的回调函数
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;  // 更新目标位置为传入的目标消息中的位置
    _goal_received = true;
    _goal_reached = false;
    ROS_INFO("Goal Received :goalCB!");
}


// Callback: Check if the car is inside the goal area or not // 检查车辆是否在目标区域内的回调函数
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
     // 如果目标已接收
    if(_goal_received)
    {
        // 计算车辆到目标点的x和y方向距离
        double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;

        // 计算车辆到目标点的欧几里得距离
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < _goalRadius)// 如果车辆到目标点的距离小于目标半径
        {
            if(start_timef) // 如果已经开始计时
            {
                tracking_etime = ros::Time::now(); // 更新结束时间为当前时间
                tracking_time_sec = tracking_etime.sec - tracking_stime.sec; // 计算跟踪时间的秒和纳秒部分
                tracking_time_nsec = tracking_etime.nsec - tracking_stime.nsec; 
                
                                
                file << "tracking time"<< "," << tracking_time_sec << "," <<  tracking_time_nsec << "\n";

                file.close();

                
                start_timef = false; // 设置开始时间标志为假
                
            }
            _goal_received = false;
            _goal_reached = true;
            _path_computed = false;
            ROS_INFO("Goal Reached !");
            cout << "tracking time: " << tracking_time_sec << "." << tracking_time_nsec << endl;

        }
    }
}


// Timer: Control Loop (closed loop nonlinear MPC)// 控制循环定时器回调函数（闭环非线性MPC）
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{          
    if(_goal_received && !_goal_reached && _path_computed ) //received goal & goal not reached   // 如果目标已接收且未到达，并且路径已计算
    {
        // 如果尚未开始计时，则记录开始时间
        if(!start_timef)
        {
            tracking_stime == ros::Time::now();
            start_timef = true;
        }
        nav_msgs::Odometry odom = _odom; // 当前里程计信息
        nav_msgs::Path odom_path = _odom_path;   // 当前路径信息
        geometry_msgs::Point goal_pos = _goal_pos; // 目标位置

        // Update system states: X=[x, y, theta, v] // 从里程计信息中提取车辆当前位置和速度
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;

         // 将ROS坐标系中的四元数转换为tf::Pose对象
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        double theta = tf::getYaw(pose.getRotation());// 车辆朝向角度
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame // 线速度


        // Update system inputs: U=[w, throttle] // 更新系统输入：U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian // 角速度
        const double throttle = _throttle; // accel: >0; brake: <0 // 油门（加速度）
        const double dt = _dt;// 采样时间间隔
        //const double Lf = _Lf;

        // Waypoints related parameters// 路径点相关参数
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);

        // Convert to the vehicle coordinate system  // 将路径点转换到车辆坐标系
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
              // 计算路径点相对于车辆当前位置的坐标
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;// 沿x轴距离
            y_veh[i] = dy * costheta - dx * sintheta;// 沿y轴距离
        }

        // Fit waypoints // 拟合路径点多项式
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        const double cte  = polyeval(coeffs, 0.0); // 计算当前位置处的cte（cross track error）

         // 调试输出多项式系数
        cout << "coeffs : " << coeffs[0] << endl;
        cout << "pow : " << pow(0.0 ,0) << endl;
        cout << "cte : " << cte << endl;

        double etheta = atan(coeffs[1]); // 计算epsi（heading error）

        // Global coordinate system about theta // 计算全局坐标系下的theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++) 
        {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }       
        
        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        double PI = 3.141592;

        // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity      // 角度转换 -pi~pi -> 0~2pi (逆时针) 以保证连续性       
        if(temp_theta <= -PI + traj_deg) 
            temp_theta = temp_theta + 2 * PI;
        
        // Implementation about theta error more precisly  // 更精确地实现theta误差
        if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
            etheta = temp_theta - traj_deg;
        else
            etheta = 0;

        cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;


        // 记录日志信息
        idx++;
        file << idx<< "," << cte << "," <<  etheta << "," << _twist_msg.linear.x << "," << _twist_msg.angular.z << "\n";
        


        // Difference bewteen current position and goal position // 计算与目标位置的偏差
        const double x_err = goal_pos.x -  odom.pose.pose.position.x;
        const double y_err = goal_pos.y -  odom.pose.pose.position.y;
        const double goal_err = sqrt(x_err*x_err + y_err*y_err);

        cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;


        // 定义状态向量
        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
              // 运动学模型用于预测实际控制时刻的车辆状态（当前时间 + 延迟dt）
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }
        
        // Solve MPC Problem // 解决MPC问题
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        ros::Time end = ros::Time::now();
        cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
              
        // MPC result (all described in car frame), output = (acceleration, w)     // MPC结果（全部描述在车体坐标系中），输出为（加速度，角速度）      
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration

        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }

        // Display the MPC predicted trajectory  // 显示MPC预测轨迹
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate      // 车体坐标系下的点  
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;

        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }     
        // publish the mpc trajectory  // 发布MPC轨迹
        _pub_mpctraj.publish(_mpc_traj);
    }
    else
    {
        _throttle = 0.0;
        _speed = 0.0;
        _w = 0;
        if(_goal_reached && _goal_received)
        {
            cout << "Goal Reached: control loop !" << endl;
        }
    }
    // publish general cmd_vel 
    if(_pub_twist_flag)
    {
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);
    }
}

/****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
