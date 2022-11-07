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
        void applyFilters(std::map<stream_index_pair, rs2::frame>& frames);
        void publish(stream_index_pair stream, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& info);

        double getStamp(rs2::frame frame);

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
    bool infra_rgb = _pnh.param("infra_rgb", false);
    if (!tf_prefix.empty()) {

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

    // disable tf
    _publish_tf = false;

    // initialize everything
    try
    {
        publishTopics();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        return false;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");
        return false;
    }

    // calculate static transforms
    rs2::stream_profile base_profile = getAProfile(_base_stream);
    for (const auto& stream : _enable)
    {
        if (stream.second)
        {
            calcAndPublishStaticTransform(stream.first, base_profile);
        }
    }

    bool override_rgb_transforms = _pnh.param("override_rgb_transforms", false);
    if (override_rgb_transforms) {
        // get relevant frames
        std::string rgb_frame = _frame_id[COLOR];
        std::string rgb_optical_frame = _optical_frame_id[COLOR];
        if (infra_rgb)
        {
            rgb_frame = _frame_id[INFRA0];
            rgb_optical_frame = _optical_frame_id[INFRA0];
        }

        // remove current rgb frames
        std::vector<geometry_msgs::TransformStamped> transforms;
        for (const auto& transform: _static_tf_msgs)
        {
            if (transform.child_frame_id != rgb_frame && transform.child_frame_id != rgb_frame)
            {
                transforms.push_back(transform);
            }
        }

        _static_tf_msgs = transforms;
    }

    // publish transforms
    if (_tf_publish_rate > 0)
    {
        _tf_t = std::shared_ptr<std::thread>(new std::thread(boost::bind(&PolledRealsenseNode::publishDynamicTransforms, this)));
    }
    else
    {
        _static_tf_broadcaster.sendTransform(_static_tf_msgs);
    }

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

double PolledRealsenseNode::getStamp(rs2::frame frame)
{
    return frameSystemTimeSec(frame);
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

    // initialize time base
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame.get_timestamp(), frame.get_frame_timestamp_domain());
    }

    // run first frame initialization
    if (frame.is<rs2::frameset>()) {
        auto frameset = frame.as<rs2::frameset>();
        for (auto it = frameset.begin(); it != frameset.end(); ++it) {
            runFirstFrameInitialization((*it).get_profile().stream_type());
        }
    }
    else if (frame.is<rs2::video_frame>()) {
        runFirstFrameInitialization(frame.get_profile().stream_type());
    }

    if (_waiting_for_frames) {
        ros::Time stamp(frameSystemTimeSec(frame));
        if (stamp < _min_stamp) {
            return;
        }
    }

    (*_frameset)[sip] = frame;

    size_t num_frames = 0;
    for (const auto& frame: *_frameset) {
        if (frame.second.is<rs2::frameset>()) {
            num_frames += frame.second.as<rs2::frameset>().size();
        }
        else {
            num_frames++;
        }
    }

    // check if the frameset is complete
    if (num_frames >= _stream_num) {
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

void PolledRealsenseNode::applyFilters(std::map<stream_index_pair, rs2::frame>& frames) {
    for (auto& frame: frames) {
        if (frame.second.is<rs2::frameset>()) {
            auto frameset = frame.second.as<rs2::frameset>();

            // Clip depth_frame for max range:
            rs2::depth_frame original_depth_frame = frameset.get_depth_frame();
            bool is_color_frame(frameset.get_color_frame());
            if (original_depth_frame && _clipping_distance > 0)
            {
                clip_depth(original_depth_frame, _clipping_distance);
            }

            ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
            for (std::vector<NamedFilter>::const_iterator filter_it = _filters.begin(); filter_it != _filters.end(); filter_it++)
            {
                ROS_DEBUG("Applying filter: %s", filter_it->_name.c_str());
                if ((filter_it->_name == "pointcloud") && (!original_depth_frame))
                    continue;
                if ((filter_it->_name == "align_to_color") && (!is_color_frame))
                    continue;
                frameset = filter_it->_filter->process(frameset);
            }

            ROS_DEBUG("List of frameset after applying filters: size: %d", static_cast<int>(frameset.size()));
        }
        else if (frame.second.is<rs2::depth_frame>() && _clipping_distance > 0) {
            clip_depth(frame.second, _clipping_distance);
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

    return processFrames(*raw_frames);
}

FramesPtr CameraDriver::getLatestFrames()
{
    std::lock_guard<std::mutex> lock(_get_frames_mutex);
    if (_latest_processed_frames)
    {
        return _latest_processed_frames;
    }
    else if (!_latest_frames)
    {
        return {};
    }

    _latest_processed_frames = processFrames(*_latest_frames);
    return _latest_processed_frames;
}

ros::Time CameraDriver::getLatestStamp()
{
    std::lock_guard<std::mutex> lock(_get_frames_mutex);

    if (!_camera) {
        return ros::Time(0);
    }

    if (!_latest_frames || _latest_frames->empty())
    {
        return ros::Time(0);
    }

    return ros::Time(std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->getStamp(_latest_frames->begin()->second));
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
    {
        std::lock_guard<std::mutex> lock(_get_frames_mutex);
        _latest_frames = frames;
        _latest_processed_frames = {};
    }

    if (_frame_callback) {
        _frame_callback();
    }
}

FramesPtr CameraDriver::processFrames(std::map<stream_index_pair, rs2::frame>& frames)
{
    auto camera = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera);

    auto buffer = _frame_buffer;
    _frame_buffer = {};

    if (!buffer) {
        buffer = std::make_shared<Frames>();
    }

    // get frame sizes before filtering
    for (const auto& frame: frames) {
        if (frame.second.is<rs2::frameset>()) {
            auto frameset = frame.second.as<rs2::frameset>();
            for (auto it = frameset.begin(); it != frameset.end(); ++it) {
                auto profile = (*it).get_profile();
                auto stream_type = profile.stream_type();
                auto stream_index = profile.stream_index();
                stream_index_pair sip{ stream_type, stream_index };
                (*buffer)[sip].transfer_bytes = (*it).get_data_size();
                if (isColor(profile)) {
                    (*buffer)[sip].transfer_bytes *= 0.666666;  // Transfered compressed using YUYV
                }
            }
        }
        else {
            auto profile = frame.second.get_profile();
            auto stream_type = profile.stream_type();
            auto stream_index = profile.stream_index();
            stream_index_pair sip{ stream_type, stream_index };
            (*buffer)[sip].transfer_bytes = frame.second.get_data_size();
            if (isColor(profile)) {
                (*buffer)[sip].transfer_bytes *= 0.666666;  // Transfered compressed using YUYV
            }
        }
    }

    // apply configured filters
    camera->applyFilters(frames);

    // copy frames
    for (const auto& frame: frames) {
        if (frame.second.is<rs2::frameset>()) {
            auto frameset = frame.second.as<rs2::frameset>();
            for (auto it = frameset.begin(); it != frameset.end(); ++it) {
                auto profile = (*it).get_profile();
                auto stream_type = profile.stream_type();
                auto stream_index = profile.stream_index();
                stream_index_pair sip{ stream_type, stream_index };
                copyFrame((*it), (*buffer)[sip]);
            }
        }
        else {
            auto profile = frame.second.get_profile();
            auto stream_type = profile.stream_type();
            auto stream_index = profile.stream_index();
            stream_index_pair sip{ stream_type, stream_index };
            copyFrame(frame.second, (*buffer)[sip]);
        }
    }

    if (_frame_callback) {
        _frame_callback();
    }

    for (const auto& frame: *buffer) {
        camera->publish(frame.first, frame.second.image, frame.second.info);
    }

    return buffer;
}

bool CameraDriver::isColor(rs2::stream_profile profile) const {
    if (profile.is<rs2::video_stream_profile>()) {
        auto format = profile.as<rs2::video_stream_profile>().format();
        if (format == RS2_FORMAT_RGB8 || format == RS2_FORMAT_BGR8) {
            return true;
        }
    }

    return false;
}

void CameraDriver::copyFrame(rs2::frame frame, ImageData& image_data) const {
    auto profile = frame.get_profile().as<rs2::video_stream_profile>();
    auto stream_type = profile.stream_type();
    auto stream_index = profile.stream_index();
    stream_index_pair sip{ stream_type, stream_index };
    if (!image_data.image) {
        image_data.image = boost::make_shared<sensor_msgs::Image>();
        auto format = profile.format();
        if (format == RS2_FORMAT_BGR8) {
            image_data.image->encoding = enc::BGR8;
            image_data.image->step = profile.width() * sizeof(uint8_t) * 3;
        }
        else if (format == RS2_FORMAT_BGRA8) {
            image_data.image->encoding = enc::BGRA8;
            image_data.image->step = profile.width() * sizeof(uint8_t) * 4;
        }
        else if (format == RS2_FORMAT_RGB8) {
            image_data.image->encoding = enc::RGB8;
            image_data.image->step = profile.width() * sizeof(uint8_t) * 3;
        }
        else if (format == RS2_FORMAT_RGBA8) {
            image_data.image->encoding = enc::RGBA8;
            image_data.image->step = profile.width() * sizeof(uint8_t) * 4;
        }
        else if (format == RS2_FORMAT_Y8) {
            image_data.image->encoding = enc::MONO8;
            image_data.image->step = profile.width() * sizeof(uint8_t);
        }
        else if (format == RS2_FORMAT_Y16) {
            image_data.image->encoding = enc::MONO16;
            image_data.image->step = profile.width() * sizeof(uint16_t);
        }
        else if (format == RS2_FORMAT_Z16) {
            image_data.image->encoding = enc::MONO16;
            image_data.image->step = profile.width() * sizeof(uint16_t);
        }
        else {
            ROS_WARN_THROTTLE(1.0, "Unsupported image format: %s", rs2_format_to_string(format));
            image_data.image = {};
        }
    }

    ros::Time stamp(std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->getStamp(frame));

    image_data.profile_fps = profile.fps();
    if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS)) {
        image_data.actual_fps = static_cast<int>(frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_FPS));
    }

    if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
        image_data.actual_exposure = static_cast<int>(frame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
    }

    auto image = image_data.image;
    image->width = profile.width();
    image->height = profile.height();
    image->header.stamp = stamp;

    size_t num_bytes = frame.get_data_size();
    if (image->data.size() != num_bytes)
    {
        image->data.resize(num_bytes);
    }
    std::memcpy(&(image->data[0]), frame.get_data(), num_bytes);

    if (!image_data.info || image_data.info->width != image->width || image_data.info->height != image->height)
    {
        image_data.info = std::dynamic_pointer_cast<PolledRealsenseNode>(_camera)->updateCameraInfo(profile);
        if (image_data.info)
        {
            image_data.info->header.stamp = stamp;
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
