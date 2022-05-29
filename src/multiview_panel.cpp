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
    //Views
    QGroupBox* view_layout_0 = new QGroupBox("View 0");
    QVBoxLayout *grp_0 = new QVBoxLayout;
    img_view_0_ = new QLabel();
    img_view_0_->setText("NO IMAGE");
    img_view_0_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    img_view_0_->setScaledContents(true);
    grp_0->addWidget(img_view_0_ );

    view_layout_0 -> setLayout(grp_0);
    view_layout_0->setAlignment(Qt::AlignHCenter);

    QGroupBox* view_layout_1 = new QGroupBox("View 1");
    QVBoxLayout *grp_1 = new QVBoxLayout;
    img_view_1_ = new QLabel();
    img_view_1_->setText("NO IMAGE");
    img_view_1_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    img_view_1_->setScaledContents(true);
    grp_1->addWidget(img_view_1_ );

    view_layout_1 -> setLayout(grp_1);
    view_layout_1->setAlignment(Qt::AlignHCenter);

    QGroupBox* view_layout_2 = new QGroupBox("View 2");
    QVBoxLayout *grp_2 = new QVBoxLayout;
    img_view_2_ = new QLabel();
    img_view_2_->setText("NO IMAGE");
    img_view_2_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    img_view_2_->setScaledContents(true);
    grp_2->addWidget(img_view_2_ );

    view_layout_2 -> setLayout(grp_2);
    view_layout_2->setAlignment(Qt::AlignHCenter);

    QGroupBox* view_layout_3 = new QGroupBox("View 3");
    QVBoxLayout *grp_3 = new QVBoxLayout;
    img_view_3_ = new QLabel();
    img_view_3_->setText("NO IMAGE");
    img_view_3_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    img_view_3_->setScaledContents(true);
    grp_3->addWidget(img_view_3_ );

    view_layout_3 -> setLayout(grp_3);
    view_layout_3->setAlignment(Qt::AlignHCenter);

    // Topics
    QHBoxLayout* topic_layout_0 = new QHBoxLayout;
    topic_layout_0->addWidget( new QLabel( "Image 0 topic:" ));
    img_topic_edit_0_ = new QLineEdit;
    topic_layout_0->addWidget( img_topic_edit_0_ );

    QHBoxLayout* topic_layout_1 = new QHBoxLayout;
    topic_layout_1->addWidget( new QLabel( "Image 1 topic:" ));
    img_topic_edit_1_ = new QLineEdit;
    topic_layout_1->addWidget( img_topic_edit_1_ );

    QHBoxLayout* topic_layout_2 = new QHBoxLayout;
    topic_layout_2->addWidget( new QLabel( "Image 2 topic:" ));
    img_topic_edit_2_ = new QLineEdit;
    topic_layout_2->addWidget( img_topic_edit_2_ );

    QHBoxLayout* topic_layout_3 = new QHBoxLayout;
    topic_layout_3->addWidget( new QLabel( "Image 3 topic:" ));
    img_topic_edit_3_ = new QLineEdit;
    topic_layout_3->addWidget( img_topic_edit_3_ );

    // Main Layout
    QGridLayout* layout = new QGridLayout;
    //img_grid
    layout->addWidget( view_layout_0 , 0, 0);
    layout->addWidget( view_layout_1 , 0, 1);
    layout->addWidget( view_layout_2 , 1, 0);
    layout->addWidget( view_layout_3 , 1, 1);

    //Topics
    layout->addLayout( topic_layout_0, 2, 0, 1, 2);   //row, col, # row, # col
    layout->addLayout( topic_layout_1, 3, 0, 1, 2);
    layout->addLayout( topic_layout_2, 4, 0, 1, 2);
    layout->addLayout( topic_layout_3, 5, 0, 1, 2);
    setLayout( layout );

    // Next we make signal connections.
    connect( img_topic_edit_0_, SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_0() ));
    connect( img_topic_edit_1_, SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_1() ));
    connect( img_topic_edit_2_, SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_2() ));
    connect( img_topic_edit_3_, SIGNAL( editingFinished() ), this, SLOT( updateImgTopic_3() ));
  }

  // QGroupBox* MultiViewPanel::createImgView(QGroupBox* view_layout, QLabel* img_view){
  //   view_layout = new QGroupBox("View 0");
  //   QVBoxLayout *grp_top = new QVBoxLayout;
  //   img_view = new QLabel();
  //   img_view->setText("NO IMAGE");
  //   img_view->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  //   img_view->setScaledContents(true);
  //   grp_top->addWidget(img_view);
  //
  //   view_layout -> setLayout(grp_top);
  //   view_layout->setAlignment(Qt::AlignHCenter);
  //   return view_layout;
  // }

  void MultiViewPanel::img2rviz(const sensor_msgs::ImageConstPtr& msg, QLabel *target_disp)
  {
    try
    {
      auto img = cv_bridge::toCvShare(msg, "rgb8")->image;
      QImage qt_img((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888); //Convert to qt format
      target_disp->setPixmap(QPixmap::fromImage(qt_img));//display the image in label that is created earlier
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
  }

  void MultiViewPanel::img0_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view_0_);
  }

  void MultiViewPanel::img1_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view_1_);
  }

  void MultiViewPanel::img2_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view_2_);
  }

  void MultiViewPanel::img3_Callback(const sensor_msgs::ImageConstPtr& msg)
  {
    img2rviz(msg, img_view_3_);
  }

  // Read the topic name from the QLineEdit
  void MultiViewPanel::updateImgTopic_0()
  {
    setTopic( img_topic_edit_0_->text(),  img_output_topic_0_, it_0_, img_sub_0_, 0);
  }

  void MultiViewPanel::updateImgTopic_1()
  {
    setTopic( img_topic_edit_1_->text(),  img_output_topic_1_, it_1_, img_sub_1_, 1);
  }

  void MultiViewPanel::updateImgTopic_2()
  {
    setTopic( img_topic_edit_2_->text(),  img_output_topic_2_, it_2_, img_sub_2_, 2);
  }

  void MultiViewPanel::updateImgTopic_3()
  {
    setTopic( img_topic_edit_3_->text(),  img_output_topic_3_, it_3_, img_sub_3_, 3);
  }

  // Set the topic name we are subscribing to.
  void MultiViewPanel::setTopic(
      const QString& new_topic,
      QString& target_topic,
      const image_transport::ImageTransport *imt,
      image_transport::Subscriber &imSub,
      const int cb_id	)
  {
    // Only take action if the name has changed.
    if( new_topic != target_topic )
    {
      target_topic = new_topic;
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
        imSub = it.subscribe(target_topic.toStdString(), 1, cb.begin()[cb_id], this);
      }
      Q_EMIT configChanged();
    }
  }

  // Save all configuration data from this panel to the given
  // Config object.  It is important here that you call save()
  // on the parent class so the class id and panel name get saved.
  void MultiViewPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "img_0", img_output_topic_0_ );
    config.mapSetValue( "img_1", img_output_topic_1_ );
    config.mapSetValue( "img_2", img_output_topic_2_ );
    config.mapSetValue( "img_3", img_output_topic_3_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void MultiViewPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "img_0", &topic ))
    {
      img_topic_edit_0_->setText( topic );
      updateImgTopic_0();
    }
    if( config.mapGetString( "img_1", &topic ))
    {
      img_topic_edit_1_->setText( topic );
      updateImgTopic_1();
    }
    if( config.mapGetString( "img_2", &topic ))
    {
      img_topic_edit_2_->setText( topic );
      updateImgTopic_2();
    }
    if( config.mapGetString( "img_3", &topic ))
    {
      img_topic_edit_3_->setText( topic );
      updateImgTopic_3();
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(miv_rviz_plugin::MultiViewPanel,rviz::Panel )
