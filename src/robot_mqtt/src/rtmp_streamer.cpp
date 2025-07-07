#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>

class RTMPStreamer {
public:
    RTMPStreamer() {
        // 初始化 FFmpeg 管道（示例参数，需调整）
        ffmpeg_cmd = "ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x760 -r 30 -i - -c:v libx264 -f flv rtmp://tx.direct.huya.com/huyalive/1199574560753-1199574560753-7484892029064985869-2399149244962-10057-A-1742712467-1?seq=1745919923386&type=simple";
        ffmpeg_proc = popen(ffmpeg_cmd.c_str(), "w");
        sub = nh.subscribe("/camera/color/image_raw", 10, &RTMPStreamer::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            fwrite(cv_ptr->image.data, 1, cv_ptr->image.total() * cv_ptr->image.elemSize(), ffmpeg_proc);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    FILE* ffmpeg_proc;
    std::string ffmpeg_cmd;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtmp_streamer");
    RTMPStreamer streamer;
    ros::spin();
    return 0;
}