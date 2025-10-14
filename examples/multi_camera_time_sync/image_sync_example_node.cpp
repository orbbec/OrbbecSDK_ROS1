#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <iomanip>

class ImageSyncNode {
 public:
  ImageSyncNode()
      : diff_sum_(0.0), count_(0), max_diff_(0.0), min_diff_(std::numeric_limits<double>::max()) {
    camera_01_color_sub_.subscribe(nh_, "/camera_01/color/image_raw", 1);
    camera_01_depth_sub_.subscribe(nh_, "/camera_01/depth/image_raw", 1);
    camera_02_color_sub_.subscribe(nh_, "/camera_02/color/image_raw", 1);
    camera_02_depth_sub_.subscribe(nh_, "/camera_02/depth/image_raw", 1);
    camera_03_color_sub_.subscribe(nh_, "/camera_03/color/image_raw", 1);
    camera_03_depth_sub_.subscribe(nh_, "/camera_03/depth/image_raw", 1);
    camera_04_color_sub_.subscribe(nh_, "/camera_04/color/image_raw", 1);
    camera_04_depth_sub_.subscribe(nh_, "/camera_04/depth/image_raw", 1);

    sync_.reset(new Sync(SyncPolicy(10), camera_01_color_sub_, camera_01_depth_sub_,
                         camera_02_color_sub_, camera_02_depth_sub_, camera_03_color_sub_,
                         camera_03_depth_sub_, camera_04_color_sub_, camera_04_depth_sub_));
    sync_->registerCallback(
        boost::bind(&ImageSyncNode::sync_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

    ROS_INFO("Image sync node started.");
  }

 private:
  ros::NodeHandle nh_;
  double diff_sum_;
  size_t count_;
  double max_diff_;
  double min_diff_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::Image> camera_01_color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_01_depth_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_02_color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_02_depth_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_03_color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_03_depth_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_04_color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> camera_04_depth_sub_;

  std::shared_ptr<Sync> sync_;

  void sync_callback(
      const sensor_msgs::ImageConstPtr &img01_c, const sensor_msgs::ImageConstPtr &img01_d,
      const sensor_msgs::ImageConstPtr &img02_c, const sensor_msgs::ImageConstPtr &img02_d,
      const sensor_msgs::ImageConstPtr &img03_c, const sensor_msgs::ImageConstPtr &img03_d,
      const sensor_msgs::ImageConstPtr &img04_c, const sensor_msgs::ImageConstPtr &img04_d) {
    std::vector<cv::Mat> images;
    std::vector<double> timestamps;
    std::vector<std::string> camera_names = {"camera_01", "camera_01", "camera_02", "camera_02",
                                             "camera_03", "camera_03", "camera_04", "camera_04"};

    std::vector<std::string> image_types = {"color", "depth", "color", "depth",
                                            "color", "depth", "color", "depth"};

    try {
      images.push_back(cv_bridge::toCvCopy(img01_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img01_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img02_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img02_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img03_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img03_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img04_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvCopy(img04_d, "16UC1")->image.clone());

      timestamps = {img01_c->header.stamp.toSec(), img01_d->header.stamp.toSec(),
                    img02_c->header.stamp.toSec(), img02_d->header.stamp.toSec(),
                    img03_c->header.stamp.toSec(), img03_d->header.stamp.toSec(),
                    img04_c->header.stamp.toSec(), img04_d->header.stamp.toSec()};
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    for (size_t i = 0; i < images.size(); i++) {
      if (images[i].channels() == 1) {
        cv::Mat tmp;
        images[i].convertTo(tmp, CV_8U, 255.0 / 10000.0);
        cv::applyColorMap(tmp, images[i], cv::COLORMAP_JET);
      }
      std::string ts_text =
          camera_names[i] + " " + image_types[i] + " stamp: " + std::to_string(timestamps[i]);
      cv::putText(images[i], ts_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 0, 255), 2);
    }

    int margin = 10;
    int row1_height = std::max({images[0].rows, images[1].rows, images[2].rows, images[3].rows});
    int row2_height = std::max({images[4].rows, images[5].rows, images[6].rows, images[7].rows});

    int row1_width = images[0].cols + margin + images[1].cols + margin + images[2].cols + margin +
                     images[3].cols;
    int row2_width = images[4].cols + margin + images[5].cols + margin + images[6].cols + margin +
                     images[7].cols;

    int canvas_width = std::max(row1_width, row2_width);
    int canvas_height = row1_height + margin + row2_height;

    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(30, 30, 30));

    int x_offset = 0;
    int y_offset = 0;
    for (int i = 0; i < 4; ++i) {
      images[i].copyTo(canvas(cv::Rect(x_offset, y_offset, images[i].cols, images[i].rows)));
      x_offset += images[i].cols + margin;
    }

    x_offset = 0;
    y_offset = row1_height + margin;
    for (int i = 4; i < 8; ++i) {
      images[i].copyTo(canvas(cv::Rect(x_offset, y_offset, images[i].cols, images[i].rows)));
      x_offset += images[i].cols + margin;
    }

    int screen_width = 1800;
    int screen_height = 800;

    double scale_w = static_cast<double>(screen_width) / canvas.cols;
    double scale_h = static_cast<double>(screen_height) / canvas.rows;
    double scale = std::min(1.0, std::min(scale_w, scale_h));

    cv::Mat display;
    if (scale < 1.0) {
      cv::resize(canvas, display, cv::Size(), scale, scale);
    } else {
      display = canvas;
    }

    cv::imshow("Time Synced Cameras", display);
    cv::waitKey(1);

    std::cout << std::fixed;
    std::cout << "===========================================================" << std::endl;

    for (int i = 0; i < 4; i++) {
      std::cout << "Camera0" << i + 1 << " stamp: color= " << std::setprecision(6)
                << timestamps[i * 2] << "  depth= " << std::setprecision(6) << timestamps[i * 2 + 1]
                << "  Delay relative to camera01(color): color= " << std::setprecision(3)
                << (timestamps[i * 2] - timestamps[0]) * 1000.0 << " ms"
                << "  depth= " << std::setprecision(3)
                << (timestamps[i * 2 + 1] - timestamps[0]) * 1000.0 << " ms" << std::endl;
    }

    double cur = 0.0;
    double base_t = timestamps[0];  // Camera01 color
    for (size_t i = 0; i < timestamps.size(); ++i) {
      double diff = std::fabs(timestamps[i] - base_t) * 1000.0;
      cur = std::max(cur, diff);
    }
    diff_sum_ += cur;
    count_++;
    max_diff_ = std::max(max_diff_, cur);
    min_diff_ = std::min(min_diff_, cur);
    double avg_diff = diff_sum_ / count_;

    std::cout << "\nImage Timestamp Difference Statistics" << std::endl;
    std::cout << "cur: " << cur << " ms"
              << " avg: " << avg_diff << " ms"
              << " max: " << max_diff_ << " ms"
              << " min: " << min_diff_ << " ms" << std::endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_sync_node");
  ImageSyncNode node;
  ros::spin();
  return 0;
}
