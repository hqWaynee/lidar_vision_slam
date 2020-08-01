#include "stdafx.h"
#include "RealSenseSensor.h"

#ifdef REAL_SENSE

RealSenseSensor::RealSenseSensor()
{
	color_width  = 640;
	color_height = 480;
	depth_width  = 640;
	depth_height = 480;
	frame_rate   = 30;
	m_depth_scale = 1.0;
	RGBDSensor::init(depth_width, depth_height, color_width, color_height,1);
}


RealSenseSensor::~RealSenseSensor()
{

}

//! Initializes the sensor
void RealSenseSensor::createFirstConnected()
{
	// Realsense pipeline
	rs2::pipeline pipe_init;
	pipe = pipe_init;

	// Realsense device list
	rs2::context ctx;
	auto devs = ctx.query_devices();                  
	int device_num = devs.size();
	if (device_num <= 0){
		cout << "Unable to find RealSenseSensor!" << endl;
		return;
	}
	// Realsense device numbers
	cout << "RealSenseSensor found,device num: " << device_num << endl;      

	// first device
	rs2::device dev = devs[0];
	// serial number
	char serial_number[100] = { 0 };
	strcpy(serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	cout << "device serial_number: " << serial_number << endl;

	// pipeline for color and depth
	rs2::config pipe_config;
	// color stream:640*480,BGR,30 fps
	pipe_config.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, frame_rate);
	// depth stream:640*480,Z16,30 fps
	pipe_config.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, frame_rate);

	// return profile of pipeline
	rs2::pipeline_profile profile = pipe.start(pipe_config);

	// depth scale
	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	m_depth_scale = sensor.get_depth_scale();

	// data stream
	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	// intrinsics
	auto intrinDepth = depth_stream.get_intrinsics();
	auto intrinColor = color_stream.get_intrinsics();
	initializeDepthIntrinsics(intrinDepth.fx, intrinDepth.fy, intrinDepth.ppx, intrinDepth.ppy);
	initializeColorIntrinsics(intrinColor.fx, intrinColor.fy, intrinColor.ppx, intrinColor.ppy);
	// Extrinsics
	initializeDepthExtrinsics(mat4f::identity());
	initializeColorExtrinsics(mat4f::identity());

	return;
}

//! Processes the depth & color data
bool RealSenseSensor::processDepth()
{
	// be full of depth api
	rs2::frameset frames;
	if(pipe.poll_for_frames(&frames)){
		rs2::frame depth_frame = frames.first(RS2_STREAM_DEPTH);
		const void* ddata1 = depth_frame.get_data();
		const USHORT* ddata = (USHORT*)ddata1;
		float* depth = getDepthFloat();
		for (int j = 0; j < (int)getDepthWidth()*(int)getDepthHeight(); j++)
		{
			const USHORT& d = ddata[j];
			if (d == 0)
				depth[j] = -std::numeric_limits<float>::infinity();
			else
				depth[j] = (float)d * m_depth_scale;
		}

		// be full of color api
		rs2::frame color_frame = frames.first(RS2_STREAM_COLOR);
		const void* cdata1 = color_frame.get_data();
		const UCHAR* cdata = (UCHAR*)cdata1;
		for (int j = 0; j < (int)getColorWidth()*(int)getColorHeight(); j++)
		{
			m_colorRGBX[j] = vec4uc(cdata[j * 3], cdata[j * 3 + 1], cdata[j * 3 + 2], 255);
		}
	}
	return true;
}
#endif