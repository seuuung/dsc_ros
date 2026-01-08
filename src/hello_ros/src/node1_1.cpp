#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pthread.h>
#include <unistd.h> 

ros::Publisher sensor1_pub;
int16_t sensor1_value = 100; // sensor_1의 고정값

// node2의 vel_feedback2 콜백
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
        
        std_msgs::Int16 msg_sensor1;
        msg_sensor1.data = sensor1_value;
        
        ROS_INFO("sensor_1: [%d]", msg_sensor1.data);
        
        sensor1_pub.publish(msg_sensor1);
        
        // 대기 (10Hz = 10만마이크로)
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
    
    sensor1_pub = n.advertise<std_msgs::Int16>("sensor_1", 1000);
    
    ros::Subscriber sub_vel_feedback2 = n.subscribe("vel_feedback2", 1000, VelFeedback2Callback);
    
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
