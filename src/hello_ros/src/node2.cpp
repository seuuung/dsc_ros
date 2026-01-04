#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> // usleep 함수 사용을 위해 추가

// 전역 퍼블리셔 선언
ros::Publisher vel_cmd_pub;
ros::Publisher vel_feedback2_pub;

// 전역 변수
int16_t sensor_1 = 0;
int16_t sensor_2 = 0;
int16_t vel_feedback = 0;

// sensor_1을 받는 콜백 함수
void Sensor1Callback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("Node2 received sensor_1: [%d]", msg->data);
    sensor_1 = msg->data;
}

// sensor_2를 받는 콜백 함수
void Sensor2Callback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("Node2 received sensor_2: [%d]", msg->data);
    sensor_2 = msg->data;
}

// vel_feedback을 받는 콜백 함수
void VelFeedbackCallback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("Node2 received vel_feedback: [%d]", msg->data);
    vel_feedback = msg->data;
}

// 스레드 1: vel_cmd를 10Hz로 발행
static void* thread_1(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node2 thread 1 working!!!");
        
        // 메시지 생성 및 데이터 계산 (sensor_1 + sensor_2)
        std_msgs::Int16 msg_vel_cmd;
        msg_vel_cmd.data = sensor_1 + sensor_2;
        
        // 로그 출력
        ROS_INFO("vel_cmd: [%d] (sensor_1 + sensor_2)", msg_vel_cmd.data);
        
        // 토픽 발행
        vel_cmd_pub.publish(msg_vel_cmd);
        
        // 대기 (10Hz = 100ms)
        usleep(100000);
    }
    
    ErrorExit:
    return nullptr;
}

// 스레드 2: vel_feedback2를 1Hz로 발행
static void* thread_2(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node2 thread 2 working!!!");
        
        // 메시지 생성 및 데이터 계산 (vel_feedback / 10)
        std_msgs::Int16 msg_vel_feedback2;
        msg_vel_feedback2.data = vel_feedback / 10;
        
        // 로그 출력
        ROS_INFO("vel_feedback2: [%d] (vel_feedback / 10)", msg_vel_feedback2.data);
        
        // 토픽 발행
        vel_feedback2_pub.publish(msg_vel_feedback2);
        
        // 대기 (1Hz = 1초)
        usleep(1000000);
    }
    
    ErrorExit:
    return nullptr;
}

int main(int argc, char **argv)
{
    // 노드 초기화 (노드 이름: node2)
    ros::init(argc, argv, "node2");
    ros::NodeHandle n;
    
    // 서브스크라이버 선언
    ros::Subscriber sub_sensor1 = n.subscribe("sensor_1", 1000, Sensor1Callback);
    ros::Subscriber sub_sensor2 = n.subscribe("sensor_2", 1000, Sensor2Callback);
    ros::Subscriber sub_vel_feedback = n.subscribe("vel_feedback", 1000, VelFeedbackCallback);
    
    // 퍼블리셔 선언
    vel_cmd_pub = n.advertise<std_msgs::Int16>("vel_cmd", 1000);
    vel_feedback2_pub = n.advertise<std_msgs::Int16>("vel_feedback2", 1000);
    
    // pthread 배열 선언
    pthread_t th[2];
    
    // 스레드 생성
    if (pthread_create(&th[0], nullptr, thread_1, nullptr) != 0) {
        std::perror("pthread_create 0");
        std::exit(1);
    }
    
    if (pthread_create(&th[1], nullptr, thread_2, nullptr) != 0) {
        std::perror("pthread_create 1");
        std::exit(1);
    }
    
    // 스레드 분리
    if (pthread_detach(th[0]) != 0) {
        std::perror("pthread_detach 0");
        std::exit(1);
    }
    
    if (pthread_detach(th[1]) != 0) {
        std::perror("pthread_detach 1");
        std::exit(1);
    }
    
    // 콜백 함수 대기
    ros::spin();
    
    return 0;
}
