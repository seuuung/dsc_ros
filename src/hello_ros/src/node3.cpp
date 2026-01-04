#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> // usleep 함수 사용을 위해 추가

// 전역 퍼블리셔 선언
ros::Publisher vel_feedback_pub;

// 전역 변수
int16_t vel_cmd = 0;

// vel_cmd를 받는 콜백 함수
void VelCmdCallback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("Node3 received vel_cmd: [%d]", msg->data);
    vel_cmd = msg->data;
}

// 스레드 1: vel_feedback을 10Hz로 발행
static void* thread_1(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node3 thread 1 working!!!");
        
        // 메시지 생성 및 데이터 계산 (vel_cmd + 100)
        std_msgs::Int16 msg_vel_feedback;
        msg_vel_feedback.data = vel_cmd + 100;
        
        // 로그 출력
        ROS_INFO("vel_feedback: [%d] (vel_cmd + 100)", msg_vel_feedback.data);
        
        // 토픽 발행
        vel_feedback_pub.publish(msg_vel_feedback);
        
        // 대기 (10Hz = 100ms)
        usleep(100000);
    }
    
    ErrorExit:
    return nullptr;
}

int main(int argc, char **argv)
{
    // 노드 초기화 (노드 이름: node3)
    ros::init(argc, argv, "node3");
    ros::NodeHandle n;
    
    // 서브스크라이버 선언
    ros::Subscriber sub_vel_cmd = n.subscribe("vel_cmd", 1000, VelCmdCallback);
    
    // 퍼블리셔 선언 (vel_feedback만!)
    vel_feedback_pub = n.advertise<std_msgs::Int16>("vel_feedback", 1000);
    
    // pthread 배열 선언
    pthread_t th[1];
    
    // 스레드 생성 (스레드 1개만!)
    if (pthread_create(&th[0], nullptr, thread_1, nullptr) != 0) {
        std::perror("pthread_create 0");
        std::exit(1);
    }
    
    // 스레드 분리
    if (pthread_detach(th[0]) != 0) {
        std::perror("pthread_detach 0");
        std::exit(1);
    }
    
    // 콜백 함수 대기
    ros::spin();
    
    return 0;
}
