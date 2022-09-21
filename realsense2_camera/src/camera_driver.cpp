#include <realsense2_camera/camera_driver.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <thread>
#include <unordered_map>

#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace realsense2_camera {

typedef std::map<stream_index_pair, rs2::frame> RawFrames;
typedef std::shared_ptr<RawFrames> RawFramesPtr;

class PolledRealsenseNode : public BaseRealSenseNode
{
    public:
        using Ptr = std::shared_ptr<PolledRealsenseNode>;
        using FrameCallback = std::function<void(RawFramesPtr)>;

        PolledRealsenseNode(ros::NodeHandle nh, ros::NodeHandle pnh, rs2::device dev, const std::string& serial_no);
        virtual ~PolledRealsenseNode();

        bool initialize(const std::string& tf_prefix);
        RawFramesPtr getFrames(int timeout_ms, const ros::Time& stamp);
        void setCallback(FrameCallback callback);
        sensor_msgs::CameraInfo::Ptr updateCameraInfo(const rs2::video_stream_profile& profile);
        void publish(stream_index_pair stream, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& info);

        void stopStreams();
        void startStreams();

        static CameraDriver::Ptr find(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& serial_no);
    protected:
        rs2::context _context;
        std::unordered_map<std::string, rs2::sensor> _enabled_sensors;
        std::mutex _frameset_mutex;
        std::condition_variable _frameset_condition;
        bool _is_streaming = false;
        bool _waiting_for_frames = false;
        ros::Time _min_stamp = ros::Time(0);
        size_t _stream_num = 0;
        RawFramesPtr _frameset = {};
        FrameCallback _frame_callback;

        virtual void frame_callback(rs2::frame frame) override;
};

PolledRealsenseNode::PolledRealsenseNode(ros::NodeHandle nh, ros::NodeHandle pnh, rs2::device dev, const std::string& serial_no) :
    BaseRealSenseNode(nh, pnh, dev, serial_no)
{
}

PolledRealsenseNode::~PolledRealsenseNode()
{
    startStreams();
}

bool PolledRealsenseNode::initialize(const std::string& tf_prefix)
{
    if (!tf_prefix.empty()) {

        bool infra_rgb = _pnh.param("infra_rgb", false);

        std::string param_value;
        if (!_pnh.getParam("base_frame_id", param_value))
        {
            _pnh.setParam("base_frame_id", tf_prefix + "_link");
        }

        if (!_pnh.getParam("odom_frame_id", param_value))
        {
            _pnh.setParam("odom_frame_id", tf_prefix + "_odom_frame");
        }

        std::vector<stream_index_pair> streams(realsense2_camera::IMAGE_STREAMS);
        streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
        for (auto& stream : streams)
        {
            std::string stream_name = _stream_name[stream.first] + (stream.second > 0 ? std::to_string(stream.second) : "");
            std::string frame_name = stream_name;
            if (infra_rgb && stream_name == "infra")
            {
                frame_name = "rgb";
            }

            if (!_pnh.getParam(stream_name + "_frame_id", param_value))
            {
                _pnh.setParam(stream_name + "_frame_id", tf_prefix + "_" + frame_name + "_frame");
            }
            if (!_pnh.getParam(stream_name + "_optical_frame_id", param_value))
            {
                _pnh.setParam(stream_name + "_optical_frame_id", tf_prefix + "_" + frame_name + "_optical_frame");
            }
        }

        auto stream = realsense2_camera::COLOR;
        std::string stream_name = _stream_name[stream.first] + (stream.second > 0 ? std::to_string(stream.second) : "");
        if (!_pnh.getParam("aligned_depth_to_" + stream_name + "_frame_id", param_value))
        {
            _pnh.setParam("aligned_depth_to_" + stream_name + "_frame_id", tf_prefix + "_aligned_depth_to_" + stream_name + "_frame");
        }
    }

    publishTopics();

    _stream_num = _enabled_profiles.size();
    for (const auto& profile : _enabled_profiles)
    {
        std::string module_name = _sensors[profile.first].get_info(RS2_CAMERA_INFO_NAME);
        _enabled_sensors[module_name] = _sensors[profile.first];
    }

    _is_streaming = true;
    stopStreams();

    // TODO fail on unsupported config

    return true;
}

RawFramesPtr PolledRealsenseNode::getFrames(int timeout_ms, const ros::Time& stamp)
{
    if (_is_streaming)
    {
        ROS_WARN("Unable to get frames, device is already streaming");
        return {};
    }

    // enable polling mode
    {
        std::lock_guard<std::mutex> lock(_frameset_mutex);
        _waiting_for_frames = true;
        _min_stamp = stamp;
    }

    startStreams();

    // wait up to the specified timeout for notification that the frameset has
    // been populated
    RawFramesPtr frames;
    auto now = std::chrono::system_clock::now();
    auto timeout = now + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(_frameset_mutex);
    if(_frameset_condition.wait_until(lock, timeout, [this](){
        return _frameset && _frameset->size() >= _stream_num;
    }))
    {
        // copy a reference to the class level frameset
        frames = _frameset;
    }
    else {
        ROS_WARN("Timed out waiting for frame data.  Timeout: %dms", timeout_ms);
    }

    // clear the class level frameset
    _frameset = std::make_shared<RawFrames>();
    lock.unlock();

    stopStreams();

    // disable polling mode
    {
        std::lock_guard<std::mutex> lock(_frameset_mutex);
        _waiting_for_frames = false;
    }

    return frames;
}

sensor_msgs::CameraInfo::Ptr PolledRealsenseNode::updateCameraInfo(const rs2::video_stream_profile& profile)
{
    updateStreamCalibData(profile);

    auto stream_type = profile.stream_type();
    auto stream_index = profile.stream_index();
    stream_index_pair sip = { stream_type, stream_index };

    auto info_it = _camera_info.find(sip);
    if (info_it == _camera_info.end())
    {
        return {};
    }

    return boost::make_shared<sensor_msgs::CameraInfo>(info_it->second);
}

void PolledRealsenseNode::publish(stream_index_pair stream, const sensor_msgs::Image::ConstPtr& image,
                                  const sensor_msgs::CameraInfo::ConstPtr& info)
{
    auto image_pub_it = _image_publishers.find(stream);
    if (image_pub_it != _image_publishers.end() && image_pub_it->second.first.getNumSubscribers() > 0)
    {
        image_pub_it->second.first.publish(image);
    }

    auto info_pub_it = _info_publisher.find(stream);
    if (info_pub_it != _info_publisher.end() && info_pub_it->second.getNumSubscribers() > 0)
    {
        info_pub_it->second.publish(info);
    }
}

void PolledRealsenseNode::stopStreams()
{
    if (_is_streaming)
    {
        for (auto& sensor: _enabled_sensors)
        {
            sensor.second.stop();
        }
    }

    _is_streaming = false;
}

void PolledRealsenseNode::startStreams()
{
    if (!_is_streaming)
    {
        for (auto& sensor: _enabled_sensors)
        {
            sensor.second.start([this](rs2::frame frame){ frame_callback(frame); });
        }
    }

    _is_streaming = true;
}

void PolledRealsenseNode::setCallback(FrameCallback callback)
{
    _frame_callback = callback;
}

void PolledRealsenseNode::frame_callback(rs2::frame frame)
{
    std::lock_guard<std::mutex> frame_lock(_frameset_mutex);

    if (!_frameset) {
        _frameset = std::make_shared<RawFrames>();
    }

    auto stream_type = frame.get_profile().stream_type();
    auto stream_index = frame.get_profile().stream_index();
    stream_index_pair sip{stream_type,stream_index};

    initializeTimeBase(frame);

    if (_waiting_for_frames) {
        ros::Time stamp(frameSystemTimeSec(frame));
        if (stamp < _min_stamp) {
            return;
        }
    }

    (*_frameset)[sip] = frame;

    // check if the frameset is complete
    if (_frameset->size() >= _stream_num) {
        if (_waiting_for_frames) {
            // notify condition
            _frameset_condition.notify_all();
        }
        else {
            // call aggregated callback
            if (_frame_callback) {
                _frame_callback(_frameset);
            }
            _frameset = std::make_shared<RawFrames>();
        }
    }
}

CameraDriver::CameraDriver(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& serial_no) :
    _nh(nh),
    _pnh(pnh),
    _serial_no(serial_no)
{
}

std::string CameraDriver::getSerialNo() const
{
    return _serial_no;
}

void CameraDriver::reset()
{
    _camera = {};
    if (_device) {
        try {
            _device.hardware_reset();
            _device = rs2::device();
        }
        catch(const std::exception& ex) {
            ROS_ERROR("An exception has been thrown trying reset device <%s>: %s", _serial_no.c_str(), ex.what());
        }
    }
}

void CameraDriver::setCallback(FrameCallback callback)
{
    _frame_callback = callback;
}

FramesPtr CameraDriver::getFrames(double timeout, ros::Time stamp)
{
    std::lock_guard<std::mutex> lock(_get_frames_mutex);

    if (!_camera)
    {
        return  {};
    }

    auto raw_frames = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->getFrames(timeout * 1000, stamp);
    if (!raw_frames)
    {
        return {};
    }

    auto buffer = _frame_buffer;
    _frame_buffer = {};

    if (!buffer) {
        buffer = std::make_shared<Frames>();
    }
    copyFrames(*raw_frames, *buffer);

    auto camera = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera);
    for (const auto& frame: *buffer)
    {
        camera->publish(frame.first, frame.second.image, frame.second.info);
    }

    return buffer;
}

void CameraDriver::setFrameBuffer(FramesPtr frame_buffer)
{
    _frame_buffer = frame_buffer;
}

void CameraDriver::start()
{
    if (!_camera)
    {
        return;
    }
    std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->startStreams();
}

void CameraDriver::stop()
{
    if (!_camera)
    {
        return;
    }
    std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->stopStreams();
}

double CameraDriver::getProjectorTemperature() const
{
    if (!_camera)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // https://github.com/IntelRealSense/librealsense/issues/866#issuecomment-357461253
    auto dbg = _device.as<rs2::debug_protocol>();
    std::vector<uint8_t> cmd = { 0x14, 0, 0xab, 0xcd, 0x2a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    auto res = dbg.send_and_receive_raw_data(cmd);
    return res[4];
}

size_t CameraDriver::getCorruptedFrames() const
{
    return _corrupted_frames;
}

size_t CameraDriver::getHardwareErrors() const
{
    return _hardware_errors;
}

size_t CameraDriver::getHardwareEvents() const
{
    return _hardware_events;
}

size_t CameraDriver::getTimedOutFrames() const
{
    return _hardware_events;
}

size_t CameraDriver::getUnspecifiedErrors() const
{
    return _unspecified_errors;
}

bool CameraDriver::initialize()
{
    if (!_camera)
    {
        return false;
    }

    std::string tf_prefix = _pnh.param("tf_prefix", std::string(""));
    ROS_INFO("tf_prefix: [%s]", tf_prefix.c_str());

    auto camera = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera);
    if (!camera->initialize(tf_prefix))
    {
        return false;
    }

    camera->setCallback(std::bind(&CameraDriver::handleFrames, this, std::placeholders::_1));

    // setup notifications
    for (auto&& s: _device.query_sensors())
    {
        s.set_notifications_callback([&](const rs2::notification& n)
        {
            auto category = n.get_category();
            if (category == RS2_NOTIFICATION_CATEGORY_FRAMES_TIMEOUT)
            {
                _timed_out_frames++;
            }
            else if (category == RS2_NOTIFICATION_CATEGORY_FRAME_CORRUPTED)
            {
                _corrupted_frames++;
            }
            else if (category == RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR)
            {
                _hardware_errors++;
            }
            else if (category == RS2_NOTIFICATION_CATEGORY_HARDWARE_EVENT)
            {
                _hardware_events++;
            }
            else if (category == RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR)
            {
                _unspecified_errors++;
            }
        });
    }

    return true;
}

bool CameraDriver::create() {
    bool found = false;
    rs2::device dev;

    auto devices = _context.query_devices();
    ROS_DEBUG("Found %d device(s)", devices.size());

    for (size_t i = 0; i < devices.size(); i++) {
        rs2::device device;
        try {
            device = devices[i];
        }
        catch(const std::exception& e) {
            ROS_WARN("Device %zu of %d failed with exception: %s ", i + 1, devices.size(), e.what());
            continue;
        }

        std::string sn = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        ROS_DEBUG("  Checking device: <%s>", sn.c_str());

        if (_serial_no.empty() || _serial_no == sn) {
            _serial_no = sn;
            dev = device;
            found = true;
            ROS_INFO("Found matching device: <%s>", _serial_no.c_str());
            break;
        }
    }

    if (found) {
        try {
            std::string firmware_version = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
            std::string model = dev.get_info(RS2_CAMERA_INFO_NAME);
            std::string physical_port = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
            std::string product_id = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);

            ROS_INFO("Firmware version: %s", firmware_version.c_str());
            ROS_INFO("Model: %s", model.c_str());
            ROS_INFO("Physical port: %s", physical_port.c_str());
            ROS_INFO("Product id: %s", product_id.c_str());

            auto sensors = dev.query_sensors();
            ROS_INFO("Found %zu sensor modules", sensors.size());
            for(const auto& sensor : sensors) {
                std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
                auto profiles = sensor.get_stream_profiles();
                ROS_INFO("  <%s> with %zu profiles", sensor_name.c_str(), profiles.size());
            }

            _device = dev;
            _camera = std::make_shared<PolledRealsenseNode>(_nh, _pnh, dev, _serial_no);
            return true;
        }
        catch(const std::exception& ex) {
            ROS_ERROR("An exception has been thrown trying to access device <%s>: {%s}", _serial_no.c_str(), ex.what());
        }
        catch(...) {
            ROS_ERROR("Unknown exception has occured trying to access device <%s>!", _serial_no.c_str());
        }
    }

    return false;
}

void CameraDriver::handleFrames(std::shared_ptr<std::map<stream_index_pair, rs2::frame>> frames)
{
    auto buffer = _frame_buffer;
    _frame_buffer = {};
    if (!buffer)
    {
        buffer = std::make_shared<Frames>();
    }

    copyFrames(*frames, *buffer);

    if (_frame_callback)
    {
        _frame_callback(buffer);
    }

    auto camera = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera);
    for (const auto& frame: *buffer)
    {
        camera->publish(frame.first, frame.second.image, frame.second.info);
    }
}

void CameraDriver::copyFrames(const std::map<stream_index_pair, rs2::frame>& in, Frames& out)
{
    for (const auto& frame: in)
    {
        auto profile = frame.second.get_profile().as<rs2::video_stream_profile>();
        auto stream_type = profile.stream_type();
        auto stream_index = profile.stream_index();
        stream_index_pair sip{ stream_type, stream_index };

        auto out_it = out.find(sip);
        if (out_it == out.end() || !out_it->second.image)
        {
            auto image = boost::make_shared<sensor_msgs::Image>();
            auto format = profile.format();
            if (format == RS2_FORMAT_BGR8) {
                image->encoding = enc::BGR8;
                image->step = profile.width() * sizeof(uint8_t) * 3;
            }
            else if (format == RS2_FORMAT_BGRA8) {
                image->encoding = enc::BGRA8;
                image->step = profile.width() * sizeof(uint8_t) * 4;
            }
            else if (format == RS2_FORMAT_RGB8) {
                image->encoding = enc::RGB8;
                image->step = profile.width() * sizeof(uint8_t) * 3;
            }
            else if (format == RS2_FORMAT_RGBA8) {
                image->encoding = enc::RGBA8;
                image->step = profile.width() * sizeof(uint8_t) * 4;
            }
            else if (format == RS2_FORMAT_Y8) {
                image->encoding = enc::MONO8;
                image->step = profile.width() * sizeof(uint8_t);
            }
            else if (format == RS2_FORMAT_Y16) {
                image->encoding = enc::MONO16;
                image->step = profile.width() * sizeof(uint16_t);
            }
            else if (format == RS2_FORMAT_Z16) {
                image->encoding = enc::MONO16;
                image->step = profile.width() * sizeof(uint16_t);
            }
            else {
                ROS_WARN_THROTTLE(1.0, "Unsupported image format: %s", rs2_format_to_string(format));
                continue;
            }

            out[sip].image = image;
        }

        ros::Time stamp(_camera->frameSystemTimeSec(frame.second));

        auto& image_data = out[sip];
        auto image = image_data.image;
        image->width = profile.width();
        image->height = profile.height();
        image->header.stamp = stamp;

        size_t num_bytes = frame.second.get_data_size();
        if (image->data.size() != num_bytes)
        {
            image->data.resize(num_bytes);
        }
        std::memcpy(&(image->data[0]), frame.second.get_data(), num_bytes);

        if (!image_data.info || image_data.info->width != image->width || image_data.info->height != image->height)
        {
            image_data.info = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->updateCameraInfo(profile);
            if (image_data.info)
            {
                image_data.info->header.stamp = stamp;
            }
        }
    }
}

CameraDriver::Ptr CameraDriver::find(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string& serial_no)
{
    auto driver = std::shared_ptr<CameraDriver>(new CameraDriver(nh, pnh, serial_no));
    if (driver->create()) {
        return driver;
    }

    return {};
}

}  // namespace realsense2_camera
