#ifndef CONTROL_PANEL_PLUGIN
#define CONTROL_PANEL_PLUGIN

#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/String.h>

#include <rqt_control_panel_plugin/rqt_control_panel_plugin_widget.h>

namespace rqt_control_panel_plugin {

  class control_panel_plugin : public rqt_gui_cpp::Plugin
  {
  public:
    control_panel_plugin();

    void initPlugin(qt_gui_cpp::PluginContext& context) override;
    void shutdownPlugin() override;

    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

  private:
    rqt_control_panel_plugin_widget *widget = nullptr;
  };

}

#endif // CONTROL_PANEL_PLUGIN

