#include <rqt_control_panel_plugin/rqt_control_panel_plugin.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

namespace rqt_control_panel_plugin {


  control_panel_plugin::control_panel_plugin() :
    rqt_gui_cpp::Plugin()
  {
    setObjectName("control_panel_plugin");
  }

  void rqt_control_panel_plugin::control_panel_plugin::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    widget = new rqt_control_panel_plugin_widget(getNodeHandle());
    context.addWidget(widget);
  }

  void rqt_control_panel_plugin::control_panel_plugin::shutdownPlugin()
  {

  }

  void rqt_control_panel_plugin::control_panel_plugin::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
  {

  }

  void rqt_control_panel_plugin::control_panel_plugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
  {

  }

} // end namespace rqt_control_panel_plugin

PLUGINLIB_EXPORT_CLASS(rqt_control_panel_plugin::control_panel_plugin, rqt_gui_cpp::Plugin)
