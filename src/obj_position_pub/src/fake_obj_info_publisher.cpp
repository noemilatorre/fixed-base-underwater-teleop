#include <ros/ros.h>
#include <lai_msgs/obj_info.h>
#include <lai_msgs/center.h>
#include <lai_msgs/bounding.h>  
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_obj_info_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<lai_msgs::obj_info>("obj_info", 10);

    ros::Rate rate(30.0);  // 1 Hz

    while (ros::ok()) {
        lai_msgs::obj_info msg;

        
        // Simuliamo 2 oggetti

        // Primo oggetto
        msg.labels.push_back("obj1");
        msg.confidences.push_back(0.9);
        msg.colors.push_back("rosso");
        msg.classes.push_back("cubo_rosso");

        lai_msgs::center c1;
        c1.cx = 0.3;
        c1.cy = 0.1;
        c1.cz = 0.06;
        msg.centers.push_back(c1);

        lai_msgs::bounding b1;
        b1.x1 = 0.25; b1.y1 = 0.05; b1.z1 = 0.05;
        b1.x2 = 0.35; b1.y2 = 0.15; b1.z2 = 0.07;
        msg.bboxs.push_back(b1);

        // Secondo oggetto
        msg.labels.push_back("obj2");
        msg.confidences.push_back(0.85);
        msg.colors.push_back("verde");
        msg.classes.push_back("cubo_verde");

        lai_msgs::center c2;
        c2.cx = 0.25;
        c2.cy = -0.15;
        c2.cz = 0.07;
        msg.centers.push_back(c2);

        lai_msgs::bounding b2;
        b2.x1 = 0.20; b2.y1 = -0.20; b2.z1 = 0.06;
        b2.x2 = 0.30; b2.y2 = -0.10; b2.z2 = 0.08;
        msg.bboxs.push_back(b2);

        pub.publish(msg);

        ROS_INFO("Messaggio obj_info pubblicato con 2 oggetti.");

        rate.sleep();
    }

    return 0;
}
