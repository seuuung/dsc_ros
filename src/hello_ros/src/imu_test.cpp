#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    // 1. 노드 초기화 (노드 이름: imu_test)
    ros::init(argc, argv, "imu_test");

    // 2. 노드 핸들 생성
    ros::NodeHandle n;

    // 3. 퍼블리셔 선언 (토픽 이름: imu_test, 큐 사이즈: 1000)
    ros::Publisher imu_test_pub = n.advertise<std_msgs::String>("imu_test", 1000);

    // 4. 루프 주기 설정 (10Hz = 0.1초마다 실행, usleep(100000)과 동일 효과)
    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;

        // 메시지 데이터 생성
        ss << "imu data " << count;
        msg.data = ss.str();

        // 터미널 출력 및 메시지 발행
        ROS_INFO("%s", msg.data.c_str());
        imu_test_pub.publish(msg);

        // 카운트 증가
        count++;

        // (선택 사항) 콜백 함수 처리가 필요할 경우를 대비해 관습적으로 추가
        ros::spinOnce();

        // 설정한 주기(10Hz)를 맞추기 위해 남은 시간만큼 대기
        loop_rate.sleep();
    }

    return 0;
}