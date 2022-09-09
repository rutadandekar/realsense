#pragma once

#include <map>
#include <string>

#include <realsense2_camera/base_realsense_node.h>

namespace realsense2_camera{

struct ImageData {
    sensor_msgs::Image::Ptr image;
    sensor_msgs::CameraInfo::Ptr info;
};

typedef std::map<stream_index_pair, ImageData> Frames;
typedef std::shared_ptr<Frames> FramesPtr;
typedef std::shared_ptr<const Frames> FramesConstPtr;

class CameraDriver
{
    public:
        using Ptr = std::shared_ptr<CameraDriver>;
        using FrameCallback = std::function<void(const FramesPtr&)>;

        ~CameraDriver() = default;

        std::string getSerialNo() const;
        bool initialize();
        void reset();
        void setCallback(FrameCallback callback);
        FramesPtr getFrames(double timeout);
        void setFrameBuffer(FramesPtr frame_buffer);
        void start();
        void stop();
        double getProjectorTemperature() const;
        size_t getCorruptedFrames() const;
        size_t getHardwareErrors() const;
        size_t getHardwareEvents() const;
        size_t getTimedOutFrames() const;
        size_t getUnspecifiedErrors() const;

        static CameraDriver::Ptr find(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& serial_no);

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _pnh;
        std::string _serial_no;
        rs2::context _context;
        rs2::device _device;
        std::shared_ptr<BaseRealSenseNode> _camera;
        FrameCallback _frame_callback;

        FramesPtr _frame_buffer;

        size_t _corrupted_frames = 0;
        size_t _hardware_errors = 0;
        size_t _hardware_events = 0;
        size_t _timed_out_frames = 0;
        size_t _unspecified_errors = 0;

        CameraDriver(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& serial_no);
        bool create();
        void handleFrames(std::shared_ptr<std::map<stream_index_pair, rs2::frame>> frames);
        void copyFrames(const std::map<stream_index_pair, rs2::frame>& in, Frames& out);
};

}  // namespace realsense2_camera
