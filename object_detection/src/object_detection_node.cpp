#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

class ObjectDetection : public rclcpp::Node {
public:
  ObjectDetection() : Node("object_detection") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/pi_cam/image_raw", 10, std::bind(&ObjectDetection::listener_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/pi_camera/image_annotated", 10);

    // Load YOLOv3 model
    std::string home = std::getenv("HOME");
    std::string config_path = home + "/exomy_ws/src/object_detection/config/yolov3.cfg";
    std::string weights_path = home + "/exomy_ws/src/object_detection/config/yolov3.weights";
    std::string names_path = home + "/exomy_ws/src/object_detection/config/coco.names";

    // Debug prints to verify paths
    std::cout << "Config path: " << config_path << std::endl;
    std::cout << "Weights path: " << weights_path << std::endl;
    std::cout << "Names path: " << names_path << std::endl;

    if (!file_exists(config_path) || !file_exists(weights_path) || !file_exists(names_path)) {
      RCLCPP_ERROR(this->get_logger(), "One or more files not found");
      return;
    }

    net_ = cv::dnn::readNet(weights_path, config_path);

    std::ifstream ifs(names_path);
    std::string line;
    while (getline(ifs, line)) {
      classes_.push_back(line);
    }
  }

private:
  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Perform object detection
    cv::Mat blob = cv::dnn::blobFromImage(frame, 0.00392, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outs;
    net_.forward(outs, get_output_layers(net_));

    // Process detections
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    for (size_t i = 0; i < outs.size(); ++i) {
      float* data = (float*)outs[i].data;
      for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
        cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
        cv::Point class_id_point;
        double confidence;
        cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);
        if (confidence > 0.5) {
          int center_x = (int)(data[0] * frame.cols);
          int center_y = (int)(data[1] * frame.rows);
          int width = (int)(data[2] * frame.cols);
          int height = (int)(data[3] * frame.rows);
          int x = center_x - width / 2;
          int y = center_y - height / 2;
          boxes.push_back(cv::Rect(x, y, width, height));
          class_ids.push_back(class_id_point.x);
          confidences.push_back((float)confidence);
        }
      }
    }

    // Non-max suppression to remove overlapping bounding boxes
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      cv::Rect box = boxes[idx];
      cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
      std::string label = classes_[class_ids[idx]];
      std::string caption = label + ": " + cv::format("%.2f", confidences[idx]);
      cv::putText(frame, caption, box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    // Publish the annotated image
    sensor_msgs::msg::Image::SharedPtr annotated_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    publisher_->publish(*annotated_image);
    RCLCPP_INFO(this->get_logger(), "Publishing annotated image");
  }

  std::vector<std::string> get_output_layers(const cv::dnn::Net& net) {
    static std::vector<std::string> layer_names = net.getLayerNames();
    std::vector<int> out_layers = net.getUnconnectedOutLayers();
    std::vector<std::string> out_layers_names;
    for (size_t i = 0; i < out_layers.size(); ++i) {
      out_layers_names.push_back(layer_names[out_layers[i] - 1]);
    }
    return out_layers_names;
  }

  bool file_exists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::dnn::Net net_;
  std::vector<std::string> classes_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetection>());
  rclcpp::shutdown();
  return 0;
}
