#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> // usleep 함수 사용을 위해 추가

// 전역 퍼블리셔 선언
ros::Publisher sensor2_pub;

// 전역 변수
int16_t sensor2_value = 0; // sensor_2의 초기값

// 스레드 1: sensor_2를 1Hz로 발행 (2씩 증가)
static void* thread_1(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node1-2 thread working!!!");
        
        // 메시지 생성 및 데이터 할당
        std_msgs::Int16 msg_sensor2;
        msg_sensor2.data = sensor2_value;
        
        // 로그 출력
        ROS_INFO("sensor_2: [%d]", msg_sensor2.data);
        
        // 토픽 발행
        sensor2_pub.publish(msg_sensor2);
        
        // 변수 업데이트 (2씩 증가)
        sensor2_value += 2;
        
        // 대기 (1Hz = 1초)
        usleep(1000000);
    }
    
    ErrorExit:
    return nullptr;
}

int main(int argc, char **argv)
{
    // 노드 초기화 (노드 이름: node1_2)
    ros::init(argc, argv, "node1_2");
    ros::NodeHandle n;
    
    // 퍼블리셔 선언 (토픽 이름: sensor_2, 큐 사이즈: 1000)
    sensor2_pub = n.advertise<std_msgs::Int16>("sensor_2", 1000);
    
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
