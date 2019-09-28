//#include "free_gait_core/TypeDefs.hpp"
//#include "free_gait_core/TypePrints.hpp"
//#include "quadruped_model/common/typedefs.hpp"
//#include "quadruped_model/QuadrupedModel.hpp"
//#include "free_gait_core/base_motion/BaseMotionBase.hpp"
//#include "free_gait_core/base_motion/BaseTarget.hpp"
//#include "free_gait_core/leg_motion/JointMotionBase.hpp"
//#include "free_gait_core/leg_motion/JointTrajectory.hpp"


//#include "iostream"
//namespace free_gait {
//std::ostream& operator<< (std::ostream& out, const std::unordered_map<LimbEnum, Position, EnumClassHash>& stance)
//{
//    for (const auto& limb : stance) {
//        out << limb.first << ":" << limb.second << std::endl;
//    }
//}
//}


//using namespace free_gait;
//int main(int argc, char *argv[])
//{
//    /********************************/
//    /*    Test the typedef.hpp      */
//    /********************************/
////    free_gait::Position position_1(1, 2, 3);
////    free_gait::Position position_2(4, 5, 6);
////    free_gait::Position position_3(7, 8, 9);

////    using LimbEnum = quadruped_model::QuadrupedModel::QuadrupedDescription::LimbEnum;

////    free_gait::Stance stance{{LimbEnum::RH_LEG, position_1},{LimbEnum::LF_LEG, position_2},{LimbEnum::LH_LEG, position_3}};

////    std::cout << stance.size() << std::endl;
////    for (auto& leg : stance) {
////        std::cout << leg.first << std::endl;
////        std::cout << leg.second << std::endl;
////    }
////    operator<<(std::cout, stance);

////    std::vector<Position> footholds = {{0,0,0},{1,1,1},{0,1,0}};
////    getFootholdsCounterClockwiseOrdered(stance,footholds);
////    std::cout << footholds.size() << std::endl;
////    for (auto it : footholds) {
////        std::cout << it <<std::endl;
////    }
////    std::cout << "the second time output " << std::endl;
////    operator<<(std::cout, stance);

//    /********************************/
//    /*    Test the basetarget.hpp   */
//    /********************************/

////    BaseTarget basetarget;

////    typedef typename curves::CubicHermiteSE3Curve::ValueType valuetype;
////    typedef typename curves::Time Time;
////    curves::CubicHermiteSE3Curve trajectory;

////    std::vector<Time> times;
////    std::vector<valuetype> values;

////    Position p_start(2, 2, 2);
////    RotationQuaternion r_start(1, 0, 0, 0);


////    Pose start_2(p_start, r_start);


////    Pose start;
////    start.setIdentity();

////    times.push_back(0.0);
////    values.push_back(start);

////    times.push_back(2.0);
////    values.push_back(start_2);

////    trajectory.fitCurve(times, values);
////    for (auto& i : times) {
////        std::cout << i << std::endl;
////    }

////    for (auto& i : values)
////        {
////            std::cout << i << std::endl;
////        }

//    /********************************/
//    /*    Test the Position2.hpp    */
//    /********************************/

////    Position2 position_1;
////    position_1 << 0.4, 0.25;
////    std::cout << position_1 << std::endl;
////    std::cout << position_1(0) << position_1(1) << std::endl;

//    /********************************/
//    /*    Test the unorder_map.hpp    */
//    /********************************/
////    quadruped_model::QuadrupedModel::QuadrupedDescription test_model;
////    typedef quadruped_model::QuadrupedModel::QuadrupedDescription::LimbEnum LimbEnum;
//    LimbEnum test_limb = LimbEnum::LF_LEG;
//    typedef typename curves::PolynomialSplineQuinticScalarCurve::ValueType ValueType;
//    typedef typename curves::Time Time;

//    free_gait::JointTrajectory test_trajectory(test_limb);
//    std::vector<Time> test_time{0, 1, 2, 3, 4};
//    std::unordered_map<ControlLevel, std::vector<Time>, EnumClassHash> test_time_queue = {{free_gait::ControlLevel::Position, test_time}};
//    std::vector<std::vector<ValueType>> test_value(3);
//    for (unsigned int i = 0; i < 3; i++) {
//        test_value[i].resize(3);
//    }
//    std::unordered_map<ControlLevel, std::vector<std::vector<ValueType>>, EnumClassHash> test_value_queue = {{free_gait::ControlLevel::Position, test_value}};

//    auto values = test_value_queue.at(ControlLevel::Position);
//    auto times = test_time_queue.at(ControlLevel::Position);

//    std::cout << "values.size is " << values.size() << std::endl;
//    std::cout << "values[0].size is " << values[0].size() << std::endl;

//    JointPositionsLeg startposition = {0, 0, 0};

//    for (unsigned int j = 0; j < values[0].size(); ++j) {
//        for (unsigned int i = 0; i < values.size(); ++i) {
//            values[i][j] += startposition(i);
//        }

//    }

//    return 0;
//}

#include <boost/thread.hpp>
#include <iostream>

void wait(int seconds)
{
  boost::this_thread::sleep(boost::posix_time::seconds(seconds));
}

boost::mutex mutex;

static int number = 1;

void thread()
{
    std::cout << "first thread" << std::endl;
  for (int i = 1; i < 5; ++i)
  {


    wait(1);
    mutex.lock();
    number = number + i;
    std::cout << "first Thread " << boost::this_thread::get_id() << ": " << number << std::endl;
    mutex.unlock();
  }
}

void thread2()
{
    std::cout << "second thread" << std::endl;
    for (int i = 1; i < 5; ++i)
    {

        wait(3);
        mutex.lock();
        number = number + i;
        std::cout << "second Thread " << boost::this_thread::get_id() << ": " << number << std::endl;

            mutex.unlock();
    }
}


int main()
{
  boost::thread t1(thread);
  std::cout << "waiting" << std::endl;
  std::cout << "1" << std::endl;
  boost::thread t2(thread2);
  t1.join();
  t2.join();
  std::cout << "2" << std::endl;

}

