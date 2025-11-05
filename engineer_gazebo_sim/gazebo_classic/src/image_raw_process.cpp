#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("image_process");

class ImageProcess : public rclcpp::Node
{
public:
    ImageProcess(std::string name) : Node(name)
    {
        image_raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>\
        ("/engineer/engineer_depth_camera/image_raw", 10, 
            std::bind(&ImageProcess::topic_callback, this, std::placeholders::_1));
        image_processed_pub_ = this->create_publisher<sensor_msgs::msg::Image>\
        ("/engineer/engineer_depth_camera/image_processed", 10);
    }
private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_raw =  cv_ptr->image;

        cv::Mat cv_image = image_processing(image_raw);
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image); // 灰度:MONO8, 彩色:BGR8
        img_bridge.toImageMsg(imageproc);

        image_processed_pub_->publish(imageproc);
    }

    cv::Mat image_processing(const cv::Mat image)
    {
        cv::Mat imageproc = image, imagebinary, imageboundary;
        cv::Mat anti_aliasing_image(image.size(), CV_8U, cv::Scalar(0));
        std::vector<cv::Mat> channels;
        std::vector<cv::Point2f> corners, front_corners, side_corners, front_corners_proc;
        std::vector<std::vector<cv::Point>> _contours;

        // 提取红色通道
        cv::split(image, channels);
        auto blue = channels.at(0);
        auto red = channels.at(2);
        // 二值化
        cv::threshold(red, imagebinary, 250, 255, cv::THRESH_BINARY);
        // 轮廓提取+凸包检测
        cv::findContours(imagebinary, _contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> hull(_contours.size());
        for (int i = 0; i < _contours.size(); i++)
        {
            convexHull(cv::Mat(_contours[i]), hull[i], false);
            cv::fillConvexPoly(anti_aliasing_image, hull[i], cv::Scalar(255), cv::LINE_AA);
        }

        // Shi-Tomasi角点检测
        cv::goodFeaturesToTrack(anti_aliasing_image, corners, 35, 0.01, 1);
        // 判断角点归属
        std::vector<std::vector<cv::Point2f>> contourCorners(hull.size());
        for (auto &corner : corners) 
        {
            for (size_t j = 0; j < hull.size(); j++) 
            {
                if (cv::pointPolygonTest(hull[j], corner, false) >= 0) 
                {
                    contourCorners[j].push_back(corner);
                    break;
                }
            }
        }
        auto minsize = contourCorners[0].size();
        for (int i = 0; i < contourCorners.size(); i++)
        {
            minsize = (contourCorners[i].size() < minsize) ? contourCorners[i].size() : minsize;
        }
        for (int i = 0; i < contourCorners.size(); i++)
        {
            if (contourCorners[i].size() <= minsize)
            {
                continue;
            }
            for (int j = 0; j < contourCorners[i].size(); j++)
            {
                front_corners.push_back(contourCorners[i].at(j));
            }
        }
        for (int i = 0; i < contourCorners.size(); i++)
        {
            if (contourCorners[i].size() == minsize)
            {
                side_corners = contourCorners[i];
                break;
            }
        }
        // 绘制角点+找边界角点
        front_corners_proc.push_back(front_corners.at(0));// xmin
        front_corners_proc.push_back(front_corners.at(0));// xmax
        front_corners_proc.push_back(front_corners.at(0));// ymin
        front_corners_proc.push_back(front_corners.at(0));// ymax
        for (size_t i = 1; i < front_corners.size(); i++) 
        {
            if (front_corners_proc[0].x >= front_corners[i].x)
            {
                front_corners_proc[0] = front_corners[i];
            }
            if (front_corners_proc[1].x <= front_corners[i].x)
            {
                front_corners_proc[1] = front_corners[i];
            }
            if (front_corners_proc[2].y >= front_corners[i].y)
            {
                front_corners_proc[2] = front_corners[i];
            }
            if (front_corners_proc[3].y <= front_corners[i].y)
            {
                front_corners_proc[3] = front_corners[i];
            }
            // cv::circle(imageproc, front_corners[i], 3, cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(imageproc, front_corners_proc[0], 3, cv::Scalar(0, 255, 0), 2);
        cv::circle(imageproc, front_corners_proc[1], 3, cv::Scalar(0, 255, 0), 2);
        cv::circle(imageproc, front_corners_proc[2], 3, cv::Scalar(0, 255, 0), 2);
        cv::circle(imageproc, front_corners_proc[3], 3, cv::Scalar(0, 255, 0), 2);

        return imageproc;
    }

    sensor_msgs::msg::Image imageproc;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_processed_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcess>("image_process_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
