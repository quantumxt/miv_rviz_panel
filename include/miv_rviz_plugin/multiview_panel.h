#ifndef VIEW_PANEL_H
#define VIEW_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#endif

class QLineEdit;
class QLabel;

namespace miv_rviz_plugin
{

  class MultiViewPanel: public rviz::Panel
  {
    Q_OBJECT
  public:

    MultiViewPanel( QWidget* parent = 0 );

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    void img2rviz(const sensor_msgs::ImageConstPtr& msg, QLabel *target_disp);

    void img0_Callback(const sensor_msgs::ImageConstPtr& msg);
    void img1_Callback(const sensor_msgs::ImageConstPtr& msg);
    void img2_Callback(const sensor_msgs::ImageConstPtr& msg);
    void img3_Callback(const sensor_msgs::ImageConstPtr& msg);

    // Next come a couple of public Qt slots.
    public Q_SLOTS:

    void setTopic(
      const QString& new_topic,
      QString& target_topic,
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

      // The ROS node handle.
      ros::NodeHandle nh_;
    };

  }

  #endif
