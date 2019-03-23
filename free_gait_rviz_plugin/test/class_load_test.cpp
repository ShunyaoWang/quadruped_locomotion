#include "free_gait_core/free_gait_core.hpp"
#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_rviz_plugin/FreeGaitPreviewDisplay.hpp"
#include "pluginlib_tutorials/polygon_plugins.h"
#include "pluginlib_tutorials/polygon_base.h"
using namespace free_gait_rviz_plugin;
using namespace free_gait;
namespace test_load_plugin {
class TestLoad
{
public:
  TestLoad(ros::NodeHandle& nh) : nh_(nh),
    AdapterRos_(nh_, free_gait::AdapterRos::AdapterType::Preview) {}
  ~TestLoad(){}
private:
  ros::NodeHandle nh_;
  AdapterRos AdapterRos_;

};
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "class_load_test");
  ros::NodeHandle nh("~");
  test_load_plugin::TestLoad TL(nh);
  pluginlib::ClassLoader<polygon_base::RegularPolygon> tutor_loader("pluginlib_tutorials", "polygon_base::RegularPolygon");
  tutor_loader.createInstance("pluginlib_tutorials/regular_triangle");
  //  AdapterRos adapterRos_(nh, free_gait::AdapterRos::AdapterType::Preview);
//  pluginlib::ClassLoader<AdapterBase> adapterLoader("free_gait_core", "free_gait::AdapterBase");
  pluginlib::ClassLoader<rviz::Display>* display_loader;
  display_loader = new pluginlib::ClassLoader<rviz::Display>("rviz", "rviz::Display");
//  std::unique_ptr<AdapterBase> adapter_loader;
//  std::unique_ptr<FreeGaitPreviewDisplay> display;
//  display.reset(display_loader.createUnmanagedInstance("free_gait_rviz_plugin/FreeGaitPreviewDisplay"));
//  adapter_loader.reset(adapterLoader.createUnmanagedInstance("free_gait::AdapterDummy"));
  display_loader->createUnmanagedInstance("free_gait_rviz_plugin/FreeGaitPreview");
  return 0;
}
