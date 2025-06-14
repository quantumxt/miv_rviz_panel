#ifndef MULTIVIEW_PANEL_HPP_
#define MULTIVIEW_PANEL_HPP_

#ifndef Q_MOC_RUN

#include <thread>
#include <algorithm>
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/logging.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"

#endif

class QLineEdit;
class QLabel;

namespace miv_rviz_plugin
{
  constexpr int IMG_COUNT{4};
  class MultiViewPanel: public rviz_common::Panel
  {
    Q_OBJECT
  public:

    MultiViewPanel( QWidget* parent = 0 );
    void onInitialize() override;

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz_common::Config& config );
    virtual void save( rviz_common::Config config ) const;
    void img2rviz(const sensor_msgs::msg::Image::ConstSharedPtr & msg, QLabel *target_disp);

    void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const int &img_id);

    // Next come a couple of public Qt slots.
    public Q_SLOTS:

    void setTopic(
      QLineEdit *line_edit,
      QString &target_topic,
      image_transport::Subscriber &imSub,
      const int callback_id	);

      // Here we declare some internal slots.
    protected Q_SLOTS:
      void updateImgTopic(const int &id);
      void updateImgTopic_0();
      void updateImgTopic_1();
      void updateImgTopic_2();
      void updateImgTopic_3();

    protected:
      QLineEdit* itopic_edit[4];

      // Current output topic.
      QString img_output_topic[4];

      // Image Views -> Qlabel used to display images
      QLabel* img_view[4];

      // Image subscribers
      image_transport::Subscriber img_sub[4];

      std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> rviz_node_ptr_;
      rclcpp::Node::SharedPtr node_;
    };

  }

  #endif  // MULTIVIEW_PANEL_HPP_
