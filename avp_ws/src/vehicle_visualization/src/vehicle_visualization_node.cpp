/*
用于车辆可视化模拟的ROS节点，主要功能是模拟车辆在2D平面上的运动，并将车辆的运动轨迹、位姿和3D模型可视化显示在RViz中。
1.主要功能：
- 模拟车辆运动：生成车辆在2D平面上的直线运动轨迹
2.发布可视化信息：
- 发布里程计(odometry)信息
- 发布运动路径(path)信息
- 发布TF坐标变换
- 发布3D车辆模型(mesh)
*/
#include "ros/ros.h" // 引入ROS核心功能头文件，提供ROS节点基础功能
#include <nav_msgs/Odometry.h> // 引入里程计消息类型，用于发布车辆位姿和速度信息
#include <nav_msgs/Path.h> // 引入路径消息类型，用于存储和发布车辆运动轨迹
#include <geometry_msgs/PoseStamped.h> // 引入带时间戳的位姿消息类型，用于构建路径
#include <sensor_msgs/Image.h> // 引入图像消息类型
#include <sensor_msgs/Imu.h> // 引入IMU消息类型
#include <eigen3/Eigen/Dense> // Eigen矩阵库
#include <eigen3/Eigen/Geometry> // 引入Eigen几何运算功能，用于旋转矩阵和四元数计算
#include "visualization_msgs/Marker.h" // 引入RViz可视化标记消息类型，用于显示3D模型
#include "string.h" // 引入字符串操作头文件
#include <tf/transform_broadcaster.h> // 引入TF坐标变换广播功能，用于发布坐标系关系


ros::Publisher pub_path, pub_odometry; // 声明全局发布器对象pub_path，用于发布路径信息;声明全局发布器对象pub_odometry，用于发布里程计信息
nav_msgs::Path path; // 声明全局路径消息对象path，存储历史路径
ros::Publisher meshPub; // 声明全局发布器对象，用于发布3D车辆模型


/*
这是核心发布函数，负责发布所有可视化信息，将车辆位姿信息发布到各个话题，参数：当前时间、x坐标、y坐标、偏航角(yaw)：
1.位姿转换：
- 将2D位姿(x,y,yaw)转换为3D位姿(x,y,z)和旋转矩阵
- 使用Eigen库进行坐标变换和四元数计算
2.发布里程计信息：
- 创建nav_msgs::Odometry消息
- 填充位置和姿态信息
- 通过pub_odometry发布
3.发布路径信息：
- 创建geometry_msgs::PoseStamped消息
- 添加到路径中，保持最多200个位姿点
- 通过pub_path发布
4.发布TF变换：
- 使用tf::TransformBroadcaster发布从"world"到"vehicle"的坐标变换
- 包含位置和姿态信息
5.发布3D车辆模型：
- 创建visualization_msgs::Marker消息
- 设置模型资源路径为package://vehicle_visualization/meshes/car.dae
- 调整模型姿态使其与运动方向一致
- 通过meshPub发布
*/
void Publish(const double &time, const double &x, const double &y, const double &yaw) {

    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0); // 将2D位姿转换为3D位姿，z坐标固定为0
    Eigen::Matrix3d R; // 根据偏航角创建旋转矩阵（绕Z轴旋转）
    R << cos(yaw), - sin(yaw), 0, // 旋转矩阵定义（2D平面旋转）【下面三行都是】
    sin(yaw), cos(yaw), 0,
    0, 0, 1;

    Eigen::Quaterniond q(R); // 将旋转矩阵转换为四元数表示（ROS常用姿态表示方式）

    /******************** 发布里程计信息 ********************/
    nav_msgs::Odometry odometry; // 创建里程计消息对象
    odometry.header.frame_id = "world"; // 设置参考坐标系为world
    odometry.header.stamp = ros::Time(time); // 设置时间戳

    // 设置位置信息
    odometry.pose.pose.position.x = position(0); // x坐标
    odometry.pose.pose.position.y = position(1); // y坐标
    odometry.pose.pose.position.z = position(2); // z坐标

    // 设置姿态信息（四元数）
    odometry.pose.pose.orientation.x = q.x(); // 四元数x分量
    odometry.pose.pose.orientation.y = q.y(); // 四元数y分量
    odometry.pose.pose.orientation.z = q.z(); // 四元数z分量
    odometry.pose.pose.orientation.w = q.w(); // 四元数w分量
    pub_odometry.publish(odometry); // 发布里程计消息

    /******************** 发布路径信息 ********************/
    geometry_msgs::PoseStamped pose_stamped; // 创建带时间戳的位姿消息
    pose_stamped.header.frame_id = "world"; // 设置参考坐标系
    pose_stamped.header.stamp = ros::Time(time); // 设置时间戳
    pose_stamped.pose = odometry.pose.pose; // 复制位姿信息
    path.poses.push_back(pose_stamped); // 将当前位姿添加到路径中

    // 限制路径长度，保持最多200个位姿点
    if (path.poses.size() > 200) {
        path.poses.erase(path.poses.begin()); // 删除最旧的位姿
    }
    pub_path.publish(path); // 发布路径消息

    /******************** 发布TF变换 ********************/
    static tf::TransformBroadcaster br; // 创建TF广播器(静态变量只初始化一次)
    tf::Transform transform; // 创建TF变换对象
    tf::Quaternion tf_q; // 创建TF四元数对象
    // 设置变换的平移部分
    transform.setOrigin(tf::Vector3(position(0),
                                    position(1),
                                    position(2)));

    // 设置四元数参数
    tf_q.setW(q.w()); // 设置w分量
    tf_q.setX(q.x()); // 设置x分量
    tf_q.setY(q.y()); // 设置y分量
    tf_q.setZ(q.z()); // 设置z分量
    transform.setRotation(tf_q); // 设置变换的旋转部分
    // 发布从"world"到"vehicle"的坐标变换，使用右手坐标系，Z轴向上。world为固定参考坐标系，vehicle为车辆坐标系，随车辆运动而变化
    br.sendTransform(tf::StampedTransform(transform, odometry.header.stamp,
                                          "world", "vehicle"));

    /******************** 发布3D车辆模型 ********************/
    visualization_msgs::Marker meshROS; // 创建可视化标记对象
    meshROS.header.frame_id = std::string("world"); // 设置参考坐标系
    meshROS.header.stamp = ros::Time(time); // 设置时间戳
    meshROS.ns = "mesh"; // 设置命名空间
    meshROS.id = 0; // 设置标记ID
    // 设置标记类型为网格资源
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD; // 动作为添加

    // 创建模型旋转矩阵（调整模型方向使其与常规车辆方向一致）
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, // 绕X轴旋转180度
                 0, 0, 1, // 调整Y和Z轴方向
                 0, 1, 0;

    Eigen::Quaterniond q_mesh; // 计算模型的最终旋转（车辆旋转+模型方向调整）
    q_mesh = q * rot_mesh; // 四元数乘法组合旋转
    Eigen::Vector3d t_mesh = R * Eigen:: Vector3d(1.5, 0, 0) + position; // 计算模型位置（车辆前方1.5米处）
    
    // 设置模型姿态
    meshROS.pose.orientation.w = q_mesh.w();  // 四元数w分量
    meshROS.pose.orientation.x = q_mesh.x();  // 四元数x分量
    meshROS.pose.orientation.y = q_mesh.y();  // 四元数y分量
    meshROS.pose.orientation.z = q_mesh.z();  // 四元数z分量
    
    // 设置模型位置
    meshROS.pose.position.x = t_mesh(0);  // x坐标
    meshROS.pose.position.y = t_mesh(1);  // y坐标
    meshROS.pose.position.z = t_mesh(2);  // z坐标

    // 设置模型缩放比例
    meshROS.scale.x = 1.0;  // x轴缩放
    meshROS.scale.y = 1.0;  // y轴缩放
    meshROS.scale.z = 1.0;  // z轴缩放
    
    // 设置模型颜色和透明度
    meshROS.color.a = 1.0;  // 不透明度(1.0为完全不透明)
    meshROS.color.r = 1.0;  // 红色分量(1.0为最大值)
    meshROS.color.g = 0.0;  // 绿色分量
    meshROS.color.b = 0.0;  // 蓝色分量

    // 设置模型资源路径(使用ROS package路径格式)
    std::string mesh_resource = "package://vehicle_visualization/meshes/car.dae"; // 3D mesh模型，模型需要是COLLADA格式(.dae文件)
    meshROS.mesh_resource = mesh_resource; // 指定模型文件路径
    meshPub.publish(meshROS); // 发布3D模型消息
}


// 生成车辆位姿的函数
// 参数：当前时间(输入)，x、y、yaw(输出)
/*
运动模拟：
- 车辆沿y=0.5x的直线运动
- 速度恒定为0.5米/秒
- 偏航角固定为arctan(0.5)≈26.565度
*/
void GeneratePose(const double &time, double &x, double &y, double &yaw) {
    double slope = 0.5;  // 定义运动轨迹的斜率
    double speed = 0.5;  // 定义运动速度(单位：米/秒)
    
    x = time * speed;  // 计算x坐标(匀速运动)
    y = slope * x;     // 计算y坐标(线性关系)
    
    // 计算偏航角(车辆朝向)，等于轨迹的倾斜角
    yaw = atan(slope);  // 使用反正切函数计算角度
}

// 主函数
int main(int argc, char **argv) {
    // 初始化ROS节点，节点名为"vehicle_visualization_node"
    ros::init(argc, argv, "vehicle_visualization_node");
    // 创建节点句柄，使用私有命名空间(~表示私有)
    ros::NodeHandle n("~");

    // 发布路径话题，话题名为"path"，队列长度1000
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    // 发布里程计话题，话题名为"odometry"，队列长度1000
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    // 发布3D模型话题，话题名为"vehicle"，队列长度100，latch设置为true(保持最后一条消息)
    meshPub = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    
    // 初始化路径的参考坐标系
    path.header.frame_id = "world";

    // 设置循环频率为10Hz
    ros::Rate loop_rate(10);
    
    double time = 0;  // 初始化时间变量
    double x = 0, y = 0, yaw = 0;  // 初始化位姿变量
    
    // 主循环
    while (ros::ok())  // 检查ROS是否正常运行
    {
        // 生成当前时间的车辆位姿
        GeneratePose(time, x, y, yaw);
        // 发布所有可视化信息
        Publish(time, x, y, yaw);
        
        // 处理ROS回调函数(虽然本节点没有订阅任何话题)
        ros::spinOnce();
        // 按照设定的频率休眠
        loop_rate.sleep();
        // 时间递增0.1秒(对应10Hz频率)
        time += 0.1;
    }
    
    // ROS事件循环(实际上前面的while循环不会退出，这里不会执行)
    ros::spin();
    return 0;
}