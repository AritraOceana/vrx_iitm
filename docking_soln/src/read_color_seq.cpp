#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Old Image";
// static const std::string OPENCV_OLD_WINDOW = "Old Image";
static const std::string OPENCV_NEW_WINDOW = "New Image";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("wamv/sensors/cameras/front_left_camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // convert the ROS image message to a CvImage suitable for working with OpenCV
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Do things on the opencv compatible image
        // NOTE: cv_ptr is now equivalent to cv.imread()
        cv::Mat old_image = cv_ptr->image;
        cv::Mat new_image;
    

        cv::cvtColor(old_image, new_image, cv::COLOR_RGB2HSV);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, old_image);
        cv::imshow(OPENCV_NEW_WINDOW, new_image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_sequence");
    ImageConverter ic;
    ros::spin();
    return 0;
}