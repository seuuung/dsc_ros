#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> // usleep 함수 사용을 위해 추가

// 전역 퍼블리셔 선언
ros::Publisher sensor1_pub;

// 전역 변수
int16_t sensor1_value = 100; // sensor_1의 고정값

// vel_feedback2를 받는 콜백 함수
void VelFeedback2Callback(const std_msgs::Int16::ConstPtr& msg)
{
    ROS_INFO("Node1-1 received vel_feedback2: [%d]", msg->data);
}

// 스레드 1: sensor_1을 10Hz로 발행
static void* thread_1(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node1-1 thread working!!!");
        
        // 메시지 생성 및 데이터 할당
        std_msgs::Int16 msg_sensor1;
        msg_sensor1.data = sensor1_value;
        
        // 로그 출력
        ROS_INFO("sensor_1: [%d]", msg_sensor1.data);
        
        // 토픽 발행
        sensor1_pub.publish(msg_sensor1);
        
        // 대기 (10Hz = 100ms)
        usleep(100000);
    }
    
    ErrorExit:
    return nullptr;
}

int main(int argc, char **argv)
{
    // 노드 초기화 (노드 이름: node1_1)
    ros::init(argc, argv, "node1_1");
    ros::NodeHandle n;
    
    // 퍼블리셔 선언 (토픽 이름: sensor_1, 큐 사이즈: 1000)
    sensor1_pub = n.advertise<std_msgs::Int16>("sensor_1", 1000);
    
    // 서브스크라이버 선언 (vel_feedback2 수신)
    ros::Subscriber sub_vel_feedback2 = n.subscribe("vel_feedback2", 1000, VelFeedback2Callback);
    
    // pthread 배열 선언
    pthread_t th[1];
    
    // 스레드 생성
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
