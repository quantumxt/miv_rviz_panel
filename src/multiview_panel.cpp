#include <stdio.h>

#include <QLineEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QImage>

#include "miv_rviz_plugin/multiview_panel.hpp"

namespace miv_rviz_plugin
{ 
  MultiViewPanel::MultiViewPanel(QWidget* parent):
  rviz_common::Panel( parent )
  {
    QGroupBox* view_layout_[IMG_COUNT];
    QHBoxLayout* topic_layout_[IMG_COUNT];

    for(int i{0}; i < IMG_COUNT; ++i){
      // Group View Layout
      QString labelTextGroup = QString("View %1").arg(i);
      auto groupBox = new QGroupBox(labelTextGroup);
      auto groupBoxLayout = new QVBoxLayout;

      img_view[i] = new QLabel("NO IMAGE");
      img_view[i]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
      img_view[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      img_view[i]->setScaledContents(true);
      groupBoxLayout->addWidget(img_view[i]);

      groupBox->setLayout(groupBoxLayout);
      groupBox->setAlignment(Qt::AlignHCenter);

      // Store the widget and layout
      view_layout_[i] = groupBox;

      // Edit text layout
      auto t_layout = new QHBoxLayout;
      QString labelTextEdit = QString("Image %1 topic: ").arg(i);
      auto label = new QLabel(labelTextEdit);
      itopic_edit[i] = new QLineEdit;
      t_layout->addWidget(label);
      t_layout->addWidget(itopic_edit[i]);
      topic_layout_[i] = t_layout;
    }

    // Main Layout
    QGridLayout* layout = new QGridLayout;
    for (int i = 0; i < IMG_COUNT; ++i) {
      //img_grid
      layout->addWidget(view_layout_[i], (i > 1) ? 1 : 0, (i % 2 == 0) ? 0 : 1);      
      //Topic editbox
      layout->addLayout(topic_layout_[i], i + 2, 0, 1, 2);
    }

    setLayout( layout );

    // Next we make signal connections.
    connect(itopic_edit[0], SIGNAL(editingFinished()), this, SLOT(updateImgTopic_0()));
    connect(itopic_edit[1], SIGNAL(editingFinished()), this, SLOT(updateImgTopic_1()));
    connect(itopic_edit[2], SIGNAL(editingFinished()), this, SLOT(updateImgTopic_2()));
    connect(itopic_edit[3], SIGNAL(editingFinished()), this, SLOT(updateImgTopic_3()));
  }

  void MultiViewPanel::img2rviz(const sensor_msgs::msg::Image::ConstSharedPtr & msg, QLabel *target_disp)
  {
    auto fmt{msg->encoding};
    auto img_enc{"rgb8"};                     //Default to rgb8
    auto q_format{QImage::Format_RGB888};     //QImg format

    try
    {
      namespace i_enc = sensor_msgs::image_encodings;

      if (fmt == i_enc::BGR8)
      {
        img_enc = "bgr8";
      }
      else if (fmt == i_enc::RGB8)
      {
        img_enc = "rgb8";
      }
      else if (fmt == i_enc::RGBA8)
      {
        img_enc = "rgba8";
        q_format = QImage::Format_RGBA8888;
      }
      else if (fmt == i_enc::TYPE_8UC1 || fmt == i_enc::TYPE_8SC1 || fmt == i_enc::MONO8)
      {
        img_enc = (fmt == i_enc::MONO8) ? "mono8" : "";
        q_format = QImage::Format_Grayscale8;
      } else {
        RVIZ_COMMON_LOG_ERROR("Invalid image format!");
        return;
      }
      auto img = cv_bridge::toCvShare(msg, img_enc)->image;
      QImage qt_img( static_cast<uchar*>(img.data), img.cols, img.rows, img.step, q_format); //Convert to qt format
      target_disp->setPixmap(QPixmap::fromImage(qt_img));
    }

    catch (cv_bridge::Exception& e)
    {
      std::string error_message = "Error while processing the image: ";
      error_message += e.what();
      RVIZ_COMMON_LOG_ERROR(error_message.c_str());
    }
  }

  void MultiViewPanel::img0_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    img2rviz(msg, img_view[0]);
  }

  void MultiViewPanel::img1_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    img2rviz(msg, img_view[1]);
  }

  void MultiViewPanel::img2_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    img2rviz(msg, img_view[2]);
  }

  void MultiViewPanel::img3_Callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    img2rviz(msg, img_view[3]);
  }

  // Read the topic name from the QLineEdit
  void MultiViewPanel::updateImgTopic(const int &id){
    setTopic(itopic_edit[id],  img_output_topic[id], img_sub[id], id);
  }

  void MultiViewPanel::updateImgTopic_0()
  {
    updateImgTopic(0);
  }

  void MultiViewPanel::updateImgTopic_1()
  {
    updateImgTopic(1);
  }

  void MultiViewPanel::updateImgTopic_2()
  {
    updateImgTopic(2);
  }

  void MultiViewPanel::updateImgTopic_3()
  {
    updateImgTopic(3);
  }

  // Set the topic name we are subscribing to.
  void MultiViewPanel::setTopic(
    QLineEdit * line_edit,
    QString& target_topic,
    image_transport::Subscriber &img_sub,
    const int cb_id	)
    {
      // Only take action if the name has changed.
      if( line_edit->text() != target_topic )
      {
        target_topic = line_edit->text();
        if( target_topic != "" )
        {
          if (node_ == nullptr) {
            rclcpp::NodeOptions options;
            node_ = rclcpp::Node::make_shared("mvp_img_listener", options);
          }
          auto cb = {
            &MultiViewPanel::img0_Callback,
            &MultiViewPanel::img1_Callback,
            &MultiViewPanel::img2_Callback,
            &MultiViewPanel::img3_Callback
          };
          // Sanitise data
          auto img_topic = target_topic.toStdString();
          img_topic.erase(std::remove_if(img_topic.begin(), img_topic.end(), [](char c) {
              return std::isspace(static_cast<unsigned char>(c));  // Efficient whitespace check
          }), img_topic.end());
          line_edit->setText(QString::fromStdString(img_topic));  // Update sanitized string
          std::cout << "View " << cb_id << ": Subscribing to topic [" << img_topic << "]" << std::endl;

          image_transport::ImageTransport it(node_);      // Subscribe img
          image_transport::TransportHints hints(node_.get());
          img_sub = it.subscribe(img_topic, 1, cb.begin()[cb_id], this);
        }
        Q_EMIT configChanged();

        if (!node_spinning_){
            node_spinning_ = true;

            // Spin node in a separate thread (to avoid blocking the GUI)
            node_thread_ = std::thread([this]() {
                rclcpp::spin(node_);
                node_spinning_ = false;
            });
            node_thread_.detach();
        }
      }
    }

    // Save all configuration data from this panel to the given Config object.
    void MultiViewPanel::save(rviz_common::Config config) const
    {
      rviz_common::Panel::save(config);
      for (int i = 0; i < 4; ++i) {
          QString key = QString("img_%1").arg(i);
          config.mapSetValue(key, img_output_topic[i]);
      }
    }

    // Load all configuration data for this panel from the given Config object.
    void MultiViewPanel::load(const rviz_common::Config& config)
    {
      rviz_common::Panel::load(config);
      QString topic;
      for (int i = 0; i < 4; ++i) {
        QString key = QString("img_%1").arg(i);  // Dynamically generate keys "img_0", "img_1", etc.
        
        if (config.mapGetString(key, &topic)) {
          itopic_edit[i]->setText(topic);  // Set the topic text for the respective index
          updateImgTopic(i);
        }
      }
    }
  }

  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(miv_rviz_plugin::MultiViewPanel, rviz_common::Panel)
