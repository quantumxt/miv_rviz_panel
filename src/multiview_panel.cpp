#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QImage>
#include <QTimer>

#include "miv_rviz_plugin/multiview_panel.h"

namespace miv_rviz_plugin
{
  MultiViewPanel::MultiViewPanel(QWidget* parent):
  rviz::Panel( parent )
  {
    QGroupBox* view_layout_[4];
    QVBoxLayout* view_box_layout_[4];
    QHBoxLayout* topic_layout_[4];
    char txt_view[10];
    char txt_topic[25];

    for(int i{0}; i<4; ++i){
      // Group View Layout
      sprintf(txt_view, "View %i", i);
      view_layout_[i] = new QGroupBox(txt_view);
      view_box_layout_[i] = new QVBoxLayout;
      img_view[i] = new QLabel();
      img_view[i] -> setText("NO IMAGE");
      img_view[i] -> setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
      img_view[i] -> setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      img_view[i] -> setScaledContents(true);
      view_box_layout_[i] -> addWidget(img_view[i]);

      view_layout_[i] -> setLayout(view_box_layout_[i]);
      view_layout_[i] -> setAlignment(Qt::AlignHCenter);

      // Edittext layout
      auto t_layout = new QHBoxLayout;
      sprintf(txt_topic, "Image %i topic: ", i);
      itopic_edit[i] = new QLineEdit;

      t_layout->addWidget( new QLabel( txt_topic ));
      t_layout->addWidget(itopic_edit[i]);
      topic_layout_[i] = t_layout;
    }

    // Main Layout
    QGridLayout* layout = new QGridLayout;
    //img_grid
    for(int i{0};i<4;++i)
      layout->addWidget( view_layout_[i] , (i > 1) ? 1 : 0, (i % 2 == 0) ? 0 : 1);

    //Topic editbox
    for(int j{0};j<4;++j)
      layout->addLayout(topic_layout_[j], j+2, 0, 1, 2);

    setLayout( layout );

    // Next we make signal connections.
    connect( itopic_edit[0], SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_0() ));
    connect( itopic_edit[1], SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_1() ));
    connect( itopic_edit[2], SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_2() ));
    connect( itopic_edit[3], SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_3() ));
  }

  void MultiViewPanel::img2rviz(const sensor_msgs::ImageConstPtr& msg, QLabel *target_disp)
  {
    auto fmt{msg->encoding};
    auto img_enc{"rgb8"};                     //Default to rgb8
    auto q_format{QImage::Format_RGB888};     //QImg format

    try
    {
      namespace i_enc = sensor_msgs::image_encodings;

      if (fmt == i_enc::RGBA8)
      {
        img_enc = "rgba8";
        q_format = QImage::Format_RGBA8888;
      }
      else if (fmt == i_enc::TYPE_8UC1 || fmt == i_enc::TYPE_8SC1 || fmt == i_enc::MONO8)
      {
        img_enc = (fmt == i_enc::MONO8) ? "mono8" : "";
        q_format = QImage::Format_Grayscale8;
      }
      auto img = cv_bridge::toCvShare(msg, img_enc)->image;
      QImage qt_img( static_cast<uchar*>(img.data), img.cols, img.rows, img.step, q_format); //Convert to qt format
      target_disp->setPixmap(QPixmap::fromImage(qt_img));
    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s -> QIMAGE: %i'.", fmt.c_str(), img_enc, q_format);
    }
  }

  void MultiViewPanel::img0_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view[0]);
  }

  void MultiViewPanel::img1_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view[1]);
  }

  void MultiViewPanel::img2_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view[2]);
  }

  void MultiViewPanel::img3_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view[3]);
  }

  // Read the topic name from the QLineEdit
  void MultiViewPanel::locate_set_topic(const int &id){
    setTopic( itopic_edit[id],  img_output_topic[id], it_[id], img_sub[id], id);
  }

  void MultiViewPanel::updateImgTopic_0()
  {
    locate_set_topic(0);
  }

  void MultiViewPanel::updateImgTopic_1()
  {
    locate_set_topic(1);
  }

  void MultiViewPanel::updateImgTopic_2()
  {
    locate_set_topic(2);
  }

  void MultiViewPanel::updateImgTopic_3()
  {
    locate_set_topic(3);
  }

  // Set the topic name we are subscribing to.
  void MultiViewPanel::setTopic(
    QLineEdit * line_edit,
    QString& target_topic,
    const image_transport::ImageTransport *imt,
    image_transport::Subscriber &img_sub,
    const int cb_id	)
    {
      // Only take action if the name has changed.
      if( line_edit->text() != target_topic )
      {
        target_topic = line_edit->text();
        if( target_topic != "" )
        {
          image_transport::ImageTransport it(nh_);      // Subscribe img
          imt = &it;
          auto cb = {
            &MultiViewPanel::img0_Callback,
            &MultiViewPanel::img1_Callback,
            &MultiViewPanel::img2_Callback,
            &MultiViewPanel::img3_Callback
          };
          // Sanitise data
          auto tp{target_topic.toStdString()};
          tp.erase(std::remove_if(tp.begin(), tp.end(),
                           [](char c) {
                               return (c == ' ' || c == '\n' || c == '\r' ||
                                       c == '\t' || c == '\v' || c == '\f');
                           }),
                           tp.end());
          line_edit->setText(QString::fromStdString(tp));           // Update sanitised string
          img_sub = it.subscribe(tp, 1, cb.begin()[cb_id], this);
        }
        Q_EMIT configChanged();
      }
    }

    // Save all configuration data from this panel to the given Config object.
    void MultiViewPanel::save( rviz::Config config ) const
    {
      rviz::Panel::save( config );
      char tmp[10];
      for(int i{0}; i<4; ++i){
        sprintf(tmp,"img_%i",i);
        config.mapSetValue( tmp, img_output_topic[i] );
      }
    }

    // Load all configuration data for this panel from the given Config object.
    void MultiViewPanel::load( const rviz::Config& config )
    {
      rviz::Panel::load( config );
      QString topic;
      if( config.mapGetString( "img_0", &topic ))
      {
        itopic_edit[0]->setText( topic );
        updateImgTopic_0();
      }
      if( config.mapGetString( "img_1", &topic ))
      {
        itopic_edit[1]->setText( topic );
        updateImgTopic_1();
      }
      if( config.mapGetString( "img_2", &topic ))
      {
        itopic_edit[2]->setText( topic );
        updateImgTopic_2();
      }
      if( config.mapGetString( "img_3", &topic ))
      {
        itopic_edit[3]->setText( topic );
        updateImgTopic_3();
      }
    }
  }

  #include <pluginlib/class_list_macros.h>
  PLUGINLIB_EXPORT_CLASS(miv_rviz_plugin::MultiViewPanel,rviz::Panel )
