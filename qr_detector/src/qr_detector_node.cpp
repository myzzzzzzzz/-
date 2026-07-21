#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <zbar.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

class QRDetectorNode : public rclcpp::Node
{
public:
  QRDetectorNode() : Node("qr_detector_node")
  {
    declare_parameter<std::string>("image_topic", "/image");
    declare_parameter<std::string>("qr_topic", "/qr_info");
    declare_parameter<double>("scan_hz", 30.0);
    declare_parameter<std::string>("detector_engine", "zbar");
    declare_parameter<int>("resize_width", 0);
    declare_parameter<double>("repeat_publish_interval", 0.3);
    declare_parameter<int>("min_confirmations", 2);
    declare_parameter<double>("confirmation_timeout", 0.7);

    image_topic_ = get_parameter("image_topic").as_string();
    qr_topic_ = get_parameter("qr_topic").as_string();
    scan_hz_ = std::max(1.0, get_parameter("scan_hz").as_double());
    detector_engine_ = normalize_detector_engine(get_parameter("detector_engine").as_string());
    resize_width_ = static_cast<int>(get_parameter("resize_width").as_int());
    repeat_publish_interval_ = std::max(0.0, get_parameter("repeat_publish_interval").as_double());
    min_confirmations_ = std::max(1, static_cast<int>(get_parameter("min_confirmations").as_int()));
    confirmation_timeout_ = std::max(0.1, get_parameter("confirmation_timeout").as_double());

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&QRDetectorNode::image_callback, this, std::placeholders::_1));

    qr_info_publisher_ = create_publisher<std_msgs::msg::String>(qr_topic_, rclcpp::QoS(10).transient_local());

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / scan_hz_));

    scan_timer_ = create_wall_timer(
      period,
      std::bind(&QRDetectorNode::scan_latest_image, this));

    RCLCPP_INFO(
      get_logger(), "QR Detector Node started with %s, min_confirmations=%d.",
      detector_engine_.c_str(), min_confirmations_);
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    latest_msg_ = msg;
    latest_stamp_sec_ = msg->header.stamp.sec;
    latest_stamp_nanosec_ = msg->header.stamp.nanosec;
  }

  void scan_latest_image()
  {
    if (!latest_msg_) {
      return;
    }

    if (latest_stamp_sec_ == last_processed_stamp_sec_ &&
        latest_stamp_nanosec_ == last_processed_stamp_nanosec_) {
      return;
    }

    last_processed_stamp_sec_ = latest_stamp_sec_;
    last_processed_stamp_nanosec_ = latest_stamp_nanosec_;

    cv::Mat gray = cv::imdecode(latest_msg_->data, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      return;
    }

    if (resize_width_ > 0 && gray.cols > resize_width_) {
      double scale = static_cast<double>(resize_width_) / static_cast<double>(gray.cols);
      int height = std::max(1, static_cast<int>(gray.rows * scale));
      cv::resize(gray, gray, cv::Size(resize_width_, height), 0.0, 0.0, cv::INTER_AREA);
    }

    if (!gray.isContinuous()) {
      gray = gray.clone();
    }

    if (detector_engine_ == "zbar" || detector_engine_ == "both") {
      const auto zbar_candidates = scan_with_zbar(gray);
      if (!zbar_candidates.empty()) {
        handle_qr_candidate(zbar_candidates.front(), "ZBar");
        return;
      }
    }

    if (detector_engine_ == "opencv" || detector_engine_ == "both") {
      const auto opencv_candidates = scan_with_opencv(gray);
      if (!opencv_candidates.empty()) {
        handle_qr_candidate(opencv_candidates.front(), "OpenCV");
      }
    }
  }

  std::vector<std::string> scan_with_opencv(const cv::Mat & gray)
  {
    std::vector<std::string> candidates;
    std::vector<std::string> decoded_info;
    cv::Mat points;
    std::vector<cv::Mat> straight_qrcodes;

    try {
      if (opencv_qr_detector_.detectAndDecodeMulti(gray, decoded_info, points, straight_qrcodes)) {
        for (auto data : decoded_info) {
          trim(data);
          if (is_valid_number(data)) {
            candidates.push_back(data);
          }
        }
      }

      if (candidates.empty()) {
        std::string data = opencv_qr_detector_.detectAndDecode(gray);
        trim(data);
        if (is_valid_number(data)) {
          candidates.push_back(data);
        }
      }
    } catch (const cv::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "OpenCV QR decode failed: %s", ex.what());
    }

    return candidates;
  }

  std::vector<std::string> scan_with_zbar(const cv::Mat & gray)
  {
    std::vector<std::string> candidates;
    zbar::Image image(
      gray.cols,
      gray.rows,
      "Y800",
      gray.data,
      static_cast<unsigned long>(gray.total()));

    const int count = scanner_.scan(image);
    if (count > 0) {
      for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        std::string data = symbol->get_data();
        trim(data);

        if (is_valid_number(data)) {
          candidates.push_back(data);
        }
      }
    }

    image.set_data(nullptr, 0);
    return candidates;
  }

  static void trim(std::string & text)
  {
    text.erase(
      text.begin(),
      std::find_if(text.begin(), text.end(), [](unsigned char ch) {
        return !std::isspace(ch);
      }));

    text.erase(
      std::find_if(text.rbegin(), text.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
      text.end());
  }

  bool is_valid_number(const std::string & text)
  {
    if (text.empty()) {
      return false;
    }
    if (text.size() > 4) {
      return false;
    }

    for (char c : text) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {
        return false;
      }
    }

    int value = std::stoi(text);
    return value >= 0 && value <= 9999;
  }

  std::string normalize_detector_engine(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    if (text == "zbar" || text == "both" || text == "opencv") {
      return text;
    }
    RCLCPP_WARN(
      get_logger(), "Unknown detector_engine '%s'; using zbar.", text.c_str());
    return "zbar";
  }

  void handle_qr_candidate(const std::string & data, const char * engine)
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = pending_candidate_.empty() ?
      confirmation_timeout_ + 1.0 :
      std::chrono::duration<double>(now - pending_candidate_time_).count();

    if (data != pending_candidate_ || dt > confirmation_timeout_) {
      pending_candidate_ = data;
      pending_count_ = 1;
    } else {
      ++pending_count_;
    }
    pending_candidate_time_ = now;

    if (pending_count_ < min_confirmations_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "QR candidate %s (%s), waiting for confirmation %d/%d.",
        data.c_str(), engine, pending_count_, min_confirmations_);
      return;
    }

    publish_qr(data, engine);
  }

  void publish_qr(const std::string & data, const char * engine)
  {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_publish_time_).count();

    if (data == last_data_ && dt < repeat_publish_interval_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = data;
    qr_info_publisher_->publish(msg);

    last_data_ = data;
    last_publish_time_ = now;

    RCLCPP_INFO(get_logger(), "Detected valid QR Code number: %s (%s)", data.c_str(), engine);
  }

private:
  std::string image_topic_;
  std::string qr_topic_;
  std::string detector_engine_;
  double scan_hz_;
  int resize_width_;
  double repeat_publish_interval_;
  int min_confirmations_;
  double confirmation_timeout_;

  sensor_msgs::msg::CompressedImage::SharedPtr latest_msg_;

  int32_t latest_stamp_sec_{0};
  uint32_t latest_stamp_nanosec_{0};
  int32_t last_processed_stamp_sec_{-1};
  uint32_t last_processed_stamp_nanosec_{0};

  std::string last_data_;
  std::string pending_candidate_;
  int pending_count_{0};
  std::chrono::steady_clock::time_point pending_candidate_time_{
    std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point last_publish_time_{
    std::chrono::steady_clock::time_point::min()};

  zbar::ImageScanner scanner_;
  cv::QRCodeDetector opencv_qr_detector_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_info_publisher_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
