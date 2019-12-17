/*
 * state estimation for legodom
 * yaochen
 * 2019.7.12
 *
 * 2019.7.17---log(日志文件)
 *
 */

#include <iostream>
#include <legodom.h>



using namespace std;
using namespace quadruped_odom;

int main(int argc, char **argv){
    ros::init(argc, argv, "legodom");
    ros::NodeHandle nodeHandle_("~");
    QuadrupedEstimation legodom(nodeHandle_);

    ros::Rate loop_rate(50);
    while(ros::ok()){
//        TicToc ttt;
        ROS_INFO("Enter Quadruped_estimation");
        if(legodom.ProcessSensorData()){}

        ros::spinOnce();
        loop_rate.sleep();
//        ros::Duration();
//        cout << "all cost time: " << ttt.toc() << " ms" << endl;
    }
    return 0;
}
