#ifndef MULTIVIEW_PANEL_HPP_
#define MULTIVIEW_PANEL_HPP_

#ifndef Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/logging.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#endif

class QLineEdit;
class QLabel;

namespace miv_rviz_plugin
{

  class MultiViewPanel: public rviz_common::Panel
  {
    Q_OBJECT
  public:

    MultiViewPanel( QWidget* parent = 0 );

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz_common::Config& config );
    virtual void save( rviz_common::Config config ) const;
    void img2rviz(const sensor_msgs::msg::Image::ConstSharedPtr & msg, QLabel *target_disp);

    void img0_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void img1_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void img2_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
    void img3_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

    // Next come a couple of public Qt slots.
    public Q_SLOTS:

    void setTopic(
      QLineEdit *line_edit,
      QString &target_topic,
      const image_transport::ImageTransport *imt,
      image_transport::Subscriber &imSub,
      const int cb_id	);

      // Here we declare some internal slots.
    protected Q_SLOTS:
      void locate_set_topic(const int &id);
      void updateImgTopic_0();
      void updateImgTopic_1();
      void updateImgTopic_2();
      void updateImgTopic_3();

    protected:
      // QLineEdit* img_topic_edit_0_;
      // QLineEdit* img_topic_edit_1_;
      // QLineEdit* img_topic_edit_2_;
      // QLineEdit* img_topic_edit_3_;
      QLineEdit* itopic_edit[4];

      // Current output topic.
      // QString img_output_topic_0_;
      // QString img_output_topic_1_;
      // QString img_output_topic_2_;
      // QString img_output_topic_3_;
      QString img_output_topic[4];


      // Image Views -> Qlabel used to display images
      // QLabel* img_view_0_;
      // QLabel* img_view_1_;
      // QLabel* img_view_2_;
      // QLabel* img_view_3_;
      QLabel* img_view[4];


      // Image subscribers
      // image_transport::Subscriber img_sub_0_;
      // image_transport::Subscriber img_sub_1_;
      // image_transport::Subscriber img_sub_2_;
      // image_transport::Subscriber img_sub_3_;
      image_transport::Subscriber img_sub[4];

      // image_transport::ImageTransport *it_0_;
      // image_transport::ImageTransport *it_1_;
      // image_transport::ImageTransport *it_2_;
      // image_transport::ImageTransport *it_3_;
      image_transport::ImageTransport* it_[4];

      rclcpp::Node::SharedPtr node_;
    };

  }

  #endif  // MULTIVIEW_PANEL_HPP_
