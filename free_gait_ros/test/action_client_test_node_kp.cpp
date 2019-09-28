#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_msgs/ExecuteActionActionGoal.h"
#include "free_gait_msgs/RobotState.h"
#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_msgs/Footstep.h"
#include "pluginlib/class_loader.h"

using namespace free_gait;

class ActionClientTest_kp
{
public:
    ActionClientTest_kp(const ros::NodeHandle& node_handle)
        :nodehandle_(node_handle), is_done(false), is_active(false), is_updated(false)
    {
        initialize();
        action_client_ptr.reset(new free_gait::FreeGaitActionClient(nodehandle_));
        base_pose_sub_ = nodehandle_.subscribe("gazebo/robot_states", 1, &ActionClientTest_kp::basePoseCallBack, this);
        action_client_ptr->registerCallback(boost::bind(&ActionClientTest_kp::activeCallback, this),
                                            boost::bind(&ActionClientTest_kp::feedbackCallback, this, _1),
                                            boost::bind(&ActionClientTest_kp::doneCallback, this, _1, _2));

    }

    ~ActionClientTest_kp()
    {}

    void initialize()
    {
        surface_normal.vector.x = 0.0;
        surface_normal.vector.y = 0.0;
        surface_normal.vector.z = 1.0;
        height = 0.4;
        step_number = 0;
    }
    void basePoseCallBack(const free_gait_msgs::RobotStateConstPtr& robot_state)
    {
        geometry_msgs::PoseWithCovariance base_pose_in_world_;
        base_pose_in_world_.pose = robot_state->base_pose.pose.pose;
        base_pose = Pose(Position(base_pose_in_world_.pose.position.x, base_pose_in_world_.pose.position.y,
                          base_pose_in_world_.pose.position.z),
                 RotationQuaternion(base_pose_in_world_.pose.orientation.w,
                                    base_pose_in_world_.pose.orientation.x,
                                    base_pose_in_world_.pose.orientation.y,
                                    base_pose_in_world_.pose.orientation.z));
        is_updated = true;
    }
    bool sendGoal(const free_gait_msgs::ExecuteStepsGoal& goal)
    {
        action_client_ptr->sendGoal(goal);
        return true;
    }

    void getFootholdInworld(const Position& foothold_in_base, geometry_msgs::PointStamped& foothold_in_world)
    {
        Position lf_position = base_pose.getRotation().rotate(foothold_in_base) + base_pose.getPosition();
        foothold_in_world.point.x = lf_position(0);
        foothold_in_world.point.y = lf_position(1);
        foothold_in_world.point.z = lf_position(2);
        ROS_INFO("Generate One Foothold : ");
        std::cout<<lf_position<<std::endl;
        foothold_in_world.header.frame_id = "odom";
    }

    bool isUpdated()
    {
        return is_updated;
    }
    bool isDone()
    {
        return is_done;
    }
    bool isActive()
    {
        return is_active;
    }

    void activeCallback()
    {
        is_active = true;
        is_done = false;
        ROS_INFO("Goal just went active");
    }

    void feedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
    {
        ROS_INFO("step time is : %f", feedback->phase);
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const free_gait_msgs::ExecuteStepsResult& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        is_done = true;
        is_active = false;
    }

    free_gait_msgs::Step generateStepForLeg(const std::string& leg_name, const geometry_msgs::PointStamped& foothold_in_world)
    {
        free_gait_msgs::Step step_msg;
        step_msg.id = std::to_string(step_number++);
        free_gait_msgs::BaseAuto base_auto_msg;
        base_auto_msg.height = height;
        base_auto_msg.average_angular_velocity = 0.5;
        base_auto_msg.average_linear_velocity = 0.5;
        base_auto_msg.ignore_timing_of_leg_motion = true;
        base_auto_msg.support_margin = 0.0;
        step_msg.base_auto.push_back(base_auto_msg);

        free_gait_msgs::Footstep footstep_msg;
        footstep_msg.name = leg_name;
        footstep_msg.target.point = foothold_in_world.point;
        footstep_msg.target.header.frame_id = "odom";
        footstep_msg.average_velocity = 0.15;
        footstep_msg.profile_height = 0.15;
        footstep_msg.profile_type = "triangle";
        footstep_msg.ignore_contact = false;
        footstep_msg.ignore_for_pose_adaptation = false;
        footstep_msg.surface_normal = surface_normal;
        step_msg.footstep.push_back(footstep_msg);

        return step_msg;
    }

private:
    ros::NodeHandle nodehandle_;
    ros::Subscriber base_pose_sub_;
    Pose base_pose;
    bool is_updated, is_done, is_active;
    std::unique_ptr<free_gait::FreeGaitActionClient> action_client_ptr;
    double height;
    int step_number;
    Position LF_nominal, RF_nominal, LH_nominal, RH_nominal;
    geometry_msgs::Vector3Stamped surface_normal;
    geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;

    LinearVelocity desired_linear_velocity_;
    LocalAngularVelocity desired_angular_velocity_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_client_test_node");
    ros::NodeHandle nodehandle("~");

    ActionClientTest_kp action_client_test(nodehandle);

    free_gait_msgs::ExecuteStepsGoal steps_goal;
    geometry_msgs::PointStamped lf_foot_holds, rf_foot_holds, lh_foot_holds, rh_foot_holds;
    Position lf_nominal, rf_nominal, lh_nominal, rh_nominal;
    double height = 0.4;

    lf_nominal = Position(0.4, 0.274, -height);
    rf_nominal = Position(0.4,-0.275,-height);
    lh_nominal = Position(-0.4,0.275,-height);
    rh_nominal = Position(-0.4,-0.275,-height);

    int i = 0;
    ros::Rate rate(100);
    while (ros::ok()) {
        if(!action_client_test.isUpdated()){
            ROS_INFO("wait for updating base pose");
        }else if (!action_client_test.isActive()) {
            action_client_test.getFootholdInworld(lf_nominal, lf_foot_holds);
            steps_goal.steps.push_back(action_client_test.generateStepForLeg("LF_LEG", lf_foot_holds));
            //! RH_LEG step *********************************************
            action_client_test.getFootholdInworld(rh_nominal, rh_foot_holds);
            steps_goal.steps.push_back(action_client_test.generateStepForLeg("RH_LEG", rh_foot_holds));
            //! RH_LEG step *********************************************
            action_client_test.getFootholdInworld(rf_nominal, rf_foot_holds);
            steps_goal.steps.push_back(action_client_test.generateStepForLeg("RF_LEG", rf_foot_holds));
            //! RH_LEG step *********************************************
            action_client_test.getFootholdInworld(lh_nominal, lh_foot_holds);
            steps_goal.steps.push_back(action_client_test.generateStepForLeg("LH_LEG", lh_foot_holds));

            action_client_test.sendGoal(steps_goal);
            }else {
                ROS_INFO("emmmmm");
                    }
            ros::spinOnce();
            rate.sleep();
    }
    return 0;
}

