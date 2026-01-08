#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> 

ros::Publisher sensor2_pub;
int16_t sensor2_value = 0; // sensor_2의 초기값

// 스레드 1: sensor_2를 1Hz로 발행, 2씩 증가
static void* thread_1(void* unused)
{
    while (ros::ok())
    {
        ROS_INFO("Node1-2 thread working!!!");
        
        std_msgs::Int16 msg_sensor2;
        msg_sensor2.data = sensor2_value;
        
        ROS_INFO("sensor_2: [%d]", msg_sensor2.data);
        
        sensor2_pub.publish(msg_sensor2);
        
        sensor2_value += 2;
        
        // 대기 (1Hz = 100만 마이크로)
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
    
    sensor2_pub = n.advertise<std_msgs::Int16>("sensor_2", 1000);
    
    pthread_t th[1];
    
    if (pthread_create(&th[0], nullptr, thread_1, nullptr) != 0) {
        std::perror("pthread_create 0");
        std::exit(1);
    }
    
    if (pthread_detach(th[0]) != 0) {
        std::perror("pthread_detach 0");
        std::exit(1);
    }
    
    ros::spin();
    
    return 0;
}
