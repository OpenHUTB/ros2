/*
 * smartcar_control_gazebo.cpp
 *
 * 阿克曼四轮小车键盘遥控节点：
 * 读取键盘输入，按阿克曼运动学实时解算两个后轮目标转速
 * 与两个前转向桥目标转角，并发布给 ros_control 控制器。
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <cstdio>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <thread>

// 整车参数
static const double WHEEL_RADIUS = 0.025;   // 车轮半径 (m)
static const double WHEELBASE    = 0.1885;  // 前后桥轴距 (m)
static const double WIDTH        = 0.16;    // 左右轮距 (m)
static const double MAX_SPEED    = 1.0;     // 最大车速 (m/s)
static const double MAX_ANGLE    = 0.4;     // 最大转向角 (rad)
static const double SPEED_STEP   = 0.1;
static const double ANGLE_STEP   = 0.1;

static std::mutex state_mutex;
static double current_speed = 0.0;
static double current_angle = 0.0;

// 键盘读取线程
void keyboardLoop()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  while (ros::ok())
  {
    char c = static_cast<char>(getchar());

    std::lock_guard<std::mutex> lock(state_mutex);
    switch (c)
    {
      case 'w': case 'W':   // 加速前进
        current_speed += SPEED_STEP;
        break;
      case 's': case 'S':   // 减速 / 后退
        current_speed -= SPEED_STEP;
        break;
      case 'e': case 'E':   // 前进时向左转
        current_angle += ANGLE_STEP;
        break;
      case 'q': case 'Q':   // 前进时向右转
        current_angle -= ANGLE_STEP;
        break;
      case 'z': case 'Z':   // 后退时向右转
        current_angle -= ANGLE_STEP;
        break;
      case 'c': case 'C':   // 后退时向左转
        current_angle += ANGLE_STEP;
        break;
      case ' ':             // 紧急停车
        current_speed = 0.0;
        current_angle = 0.0;
        break;
      case 'x': case 'X':   // 转向回正
        current_angle = 0.0;
        break;
      default:
        break;
    }

    if (current_speed > MAX_SPEED) current_speed = MAX_SPEED;
    if (current_speed < -MAX_SPEED) current_speed = -MAX_SPEED;
    if (current_angle > MAX_ANGLE) current_angle = MAX_ANGLE;
    if (current_angle < -MAX_ANGLE) current_angle = -MAX_ANGLE;
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "smartcar_control_gazebo");
  ros::NodeHandle nh;

  // 四个控制器指令话题
  ros::Publisher rear_left_pub =
      nh.advertise<std_msgs::Float64>("/rear_left_velocity_controller/command", 10);
  ros::Publisher rear_right_pub =
      nh.advertise<std_msgs::Float64>("/rear_right_velocity_controller/command", 10);
  ros::Publisher bridge_left_pub =
      nh.advertise<std_msgs::Float64>("/left_bridge_position_controller/command", 10);
  ros::Publisher bridge_right_pub =
      nh.advertise<std_msgs::Float64>("/right_bridge_position_controller/command", 10);

  std::thread keyboard_thread(keyboardLoop);
  keyboard_thread.detach();

  printf("\n控制小车（终端需获得焦点，使用英文输入法）：\n");
  printf("w: 加速前进   s: 减速/后退\n");
  printf("e: 前进左转   q: 前进右转\n");
  printf("z: 后退右转   c: 后退左转\n");
  printf("x: 转向回正   空格: 紧急停车   Ctrl+C: 退出\n\n");

  ros::Rate rate(10);  // 10 Hz 发布与打印
  while (ros::ok())
  {
    double speed, angle;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      speed = current_speed;
      angle = current_angle;
    }

    // 阿克曼运动解算
    double t = std::tan(angle);

    // 左右前轮转角
    double left_angle =
        std::atan(WHEELBASE * t / (WHEELBASE - 0.5 * WIDTH * t));
    double right_angle =
        std::atan(WHEELBASE * t / (WHEELBASE + 0.5 * WIDTH * t));

    // 左右后轮线速度
    double left_speed =
        speed * (1.0 - 0.5 * WIDTH * t / WHEELBASE);
    double right_speed =
        speed * (1.0 + 0.5 * WIDTH * t / WHEELBASE);

    // 发布控制量（速度控制器指令为车轮角速度 rad/s）
    std_msgs::Float64 msg;
    msg.data = left_speed / WHEEL_RADIUS;
    rear_left_pub.publish(msg);
    msg.data = right_speed / WHEEL_RADIUS;
    rear_right_pub.publish(msg);
    msg.data = left_angle;
    bridge_left_pub.publish(msg);
    msg.data = right_angle;
    bridge_right_pub.publish(msg);

    // 打印当前四个控制量
    printf("left_speed: %.2f, right_speed: %.2f, left_angle: %.2f, right_angle: %.2f\n",
           left_speed, right_speed, left_angle, right_angle);

    // 打印阿克曼内外侧解算结果
    bool left_is_inner = std::fabs(left_angle) >= std::fabs(right_angle);
    double outside_speed = left_is_inner ? right_speed : left_speed;
    double outside_angle = left_is_inner ? right_angle : left_angle;
    double inside_speed  = left_is_inner ? left_speed  : right_speed;
    double inside_angle  = left_is_inner ? left_angle  : right_angle;
    printf("outside_speed: %.2f outside_angle: %.2f inside_speed: %.2f inside_angle: %.2f\n",
           outside_speed, outside_angle, inside_speed, inside_angle);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
