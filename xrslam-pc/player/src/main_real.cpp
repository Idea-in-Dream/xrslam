#include <argparse.hpp>
#include <iostream>
#include <thread>
#include <mutex>

#include <dataset_reader.h>
#include <trajectory_writer.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include "XRSLAM.h"


#ifndef XRSLAM_PC_HEADLESS_ONLY
#include <unistd.h>
#include <thread>
#include "visualizer.h"


#define width 640 
#define height 480 
#define fps 30

std::thread visualizer_thread_;
VizElements cur_frame_viz_;

void GetShowElements(VizElements &curframe) {
    // clear
    curframe.landmarks.clear();
    curframe.landmarks_color.clear();
    // get pose
    XRSLAMPose pose_b;
    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
    curframe.camera_q =
        Eigen::Quaterniond(pose_b.quaternion[3], pose_b.quaternion[0],
                           pose_b.quaternion[1], pose_b.quaternion[2]);
    curframe.camera_p = Eigen::Vector3d(
        pose_b.translation[0], pose_b.translation[1], pose_b.translation[2]);
    // get landmarks
    XRSLAMLandmarks landmarks;
    XRSLAMGetResult(XRSLAM_RESULT_LANDMARKS, &landmarks);
    for (int i = 0; i < landmarks.num_landmarks; ++i) {
        curframe.landmarks.push_back(Eigen::Vector3f(landmarks.landmarks[i].x,
                                                     landmarks.landmarks[i].y,
                                                     landmarks.landmarks[i].z));
        // triangulated
        curframe.landmarks_color.emplace_back(0.0, 1.0, 0.0, 0.5);
    }
    // draw 2d features
    // XRSLAMFeatures features;
    // XRSLAMGetResult(XRSLAM_RESULT_FEATURES, &features);
    // for (int i = 0; i < features.num_features; ++i) {
    //     cv::Point center(features.features[i].x, features.features[i].y);
    //     cv::circle(curframe.show_frame, center, 2, cv::Scalar(0, 255, 0), 5);
    // }
    // get imu bias
    XRSLAMIMUBias imu_bias;
    XRSLAMGetResult(XRSLAM_RESULT_BIAS, &imu_bias);
    curframe.acc_bias =
        Eigen::Vector3d(imu_bias.acc_bias.data[0], imu_bias.acc_bias.data[1],
                        imu_bias.acc_bias.data[2]);
    curframe.gyro_bias =
        Eigen::Vector3d(imu_bias.gyr_bias.data[0], imu_bias.gyr_bias.data[1],
                        imu_bias.gyr_bias.data[2]);
}
#endif

// 类型定义 建一个元组类型 IMUData，用于保存 IMU（惯性测量单元）数据。元组包含三个元素：时间戳（double）、加速度计数据（XRSLAMAcceleration）和陀螺仪数据（XRSLAMGyroscope）。

typedef std::tuple<double, XRSLAMAcceleration, XRSLAMGyroscope> IMUData;

int main(int argc, char *argv[]) try {
    
    // 从命令行参数中获取输入文件路径、SLAM配置文件路径、设备配置文件路径和许可证文件路径。
    argparse::ArgumentParser program("XRSLAM Player");
    // SLAM 配置文件
    program.add_argument("-sc", "--slamconfig")
        .help("SLAM configuration YAML file.")
        .nargs(1);
    // 设备配置文件
    program.add_argument("-dc", "--deviceconfig")
        .help("Device configuration YAML file.")
        .nargs(1);
    // 许可证文件
    program.add_argument("-lc", "--license").help("License file.").nargs(1);
    // 保存轨迹为 CSV 格式
    program.add_argument("--csv").help("Save CSV-format trajectory.").nargs(1);
    // 保存轨迹为 TUM 格式
    program.add_argument("--tum").help("Save TUM-format trajectory.").nargs(1);
    // 立即播放
    program.add_argument("-p", "--play")
        .help("Start playing immediately.")
        .default_value(false)
        .implicit_value(true);
    // 输入文件
    program.add_argument("input").help("input file");

    // 解析命令行参数
    program.parse_args(argc, argv);
    
    // 从命令行参数解析器 program 中提取各个参数的值，并存储在相应的变量中。
    std::string data_path = program.get<std::string>("input");
    std::string slam_config_path = program.get<std::string>("-sc");
    std::string device_config_path = program.get<std::string>("-dc");
    std::string license_path = program.get<std::string>("-lc");
    std::string csv_output = program.get<std::string>("--csv");
    std::string tum_output = program.get<std::string>("--tum");
    bool is_play = program.get<bool>("-p");

    // create slam with configuration files 从配置文件中创建 SLAM 对象
    void *yaml_config = nullptr;
    int create_succ =
        XRSLAMCreate(slam_config_path.c_str(), device_config_path.c_str(),
                     license_path.c_str(), "XRSLAM PC", &yaml_config);
    std::cout << "create SLAM success: " << create_succ << std::endl;

#ifndef XRSLAM_PC_HEADLESS_ONLY
    Visualizer visualizer(is_play, device_config_path);
    visualizer.show();
    visualizer_thread_ = std::thread(&Visualizer::main, &visualizer);
    std::cout << "create visualizer thread ..." << std::endl;
#endif

    std::vector<std::unique_ptr<TrajectoryWriter>> outputs;
    if (csv_output.length() > 0) {
        outputs.emplace_back(std::make_unique<CsvTrajectoryWriter>(csv_output));
    }
    if (tum_output.length() > 0) {
        outputs.emplace_back(std::make_unique<TumTrajectoryWriter>(tum_output));
    }

    std::unique_ptr<DatasetReader> reader =
        DatasetReader::create_reader(data_path, yaml_config);
    if (!reader) {
        fprintf(stderr, "Cannot open \"%s\"\n", data_path.c_str());
        return EXIT_FAILURE;
    }

    bool has_gyroscope = false, has_accelerometer = false;
    outputs.emplace_back(std::make_unique<ConsoleTrajectoryWriter>());
    DatasetReader::NextDataType next_type;



    // judge whether devices is exist or not 
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    int index = 0;
    for (rs2::sensor sensor : sensors)
      if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
        ++index;
        if (index == 1) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          // D435i 不支持深度相机曝光修改
          //          if(sensor.supports(RS2_OPTION_AUTO_EXPOSURE_LIMIT))
          //              sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
        }
        if (index == 2) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          //            sensor.set_option(RS2_OPTION_EXPOSURE, 30);
        }
        if (index == 3) {
          sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
        }
      }


    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    // cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    // cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    
    std::cout << "before get frames" << std::endl;

    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流

    std::cout << "before get frames 0" << std::endl;
    


    while(1)
    {
        std::cout << "before get frames" << std::endl;
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架
        
        
        // Align to depth 
        // rs2::align align_to_depth(RS2_STREAM_DEPTH);
        // frames = align_to_depth.process(frames);
    
        // Get imu data
        if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;

            has_accelerometer = true;
            XRSLAMGyroscope acc_data;
            acc_data.data[0] = accel_sample.x;
            acc_data.data[1] = accel_sample.y;
            acc_data.data[2] = accel_sample.z;
            acc_data.timestamp = frames.get_timestamp()*1e-3;

            XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc_data);
        
        }
        if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();

            has_gyroscope = true;
            XRSLAMGyroscope gyro_data;
            gyro_data.data[0] = gyro_sample.x;
            gyro_data.data[1] = gyro_sample.y;
            gyro_data.data[2] = gyro_sample.z;
            gyro_data.timestamp = frames.get_timestamp()*1e-3;

            XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyro_data);

            std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        }
        
        std::cout << "get frames 0: " << std::to_string(frames.get_timestamp()) << std::endl;
        //Get each frame
        // rs2::frame color_frame = frames.get_color_frame();
        // rs2::frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);
        
        cv::Mat pic_left(cv::Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
        
        std::cout << "pic_left size: " << pic_left.size() << std::endl;
        namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
        cv::imwrite("pic_left.png", pic_left);
        cv::waitKey(1);

        std::cout << "get frames 1: " << std::to_string(frames.get_timestamp()) << std::endl;

        XRSLAMImage image;
        image.camera_id = 0;
        image.timeStamp = frames.get_timestamp()*1e-3;;
        image.ext = nullptr;
        image.data = pic_left.data;
        image.stride = pic_left.step[0];

        std::cout << "get frames 2: " << std::to_string(frames.get_timestamp()) << std::endl;

        XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
        
        
        if (has_accelerometer && has_gyroscope) {
            XRSLAMRunOneFrame();
            XRSLAMState state;
            XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
            

            std::cout << "get frames result:" << state << std::endl;
            
            if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
#ifndef XRSLAM_PC_HEADLESS_ONLY
            int cols = 0, rows = 0;
            reader->get_image_resolution(cols, rows);
            cv::Mat img = cv::Mat(rows, cols, CV_8UC1, image.data);
            cv::cvtColor(img, cur_frame_viz_.show_frame,
                            cv::COLOR_GRAY2BGR);
            GetShowElements(cur_frame_viz_);
            visualizer.update_frame(
                std::make_shared<VizElements>(cur_frame_viz_));
#endif
            XRSLAMPose pose_b;
            XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
            if (pose_b.timestamp > 0) {
                for (auto &output : outputs) {
                    output->write_pose(pose_b.timestamp, pose_b);
                    }
                }
            }
        }
    }
    return 0;












//     while ((next_type = reader->next()) != DatasetReader::END) {
// #ifndef XRSLAM_PC_HEADLESS_ONLY
//         if (!visualizer.is_running()) {
//             break;
//         }
// #endif
//         switch (next_type) {
//         case DatasetReader::AGAIN:
//             continue;
//         case DatasetReader::GYROSCOPE: {
//             has_gyroscope = true;
//             auto [t, gyro] = reader->read_gyroscope();
//             std::cout << "gyro: t" << std::to_string(t) << std::endl;
//             XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyro);
//         } break;
//         case DatasetReader::ACCELEROMETER: {
//             has_accelerometer = true;
//             auto [t, acc] = reader->read_accelerometer();
//             std::cout << "acc: t" << std::to_string(t) << std::endl;
//             XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
//         } break;
//         case DatasetReader::CAMERA: {
//             auto [t, img] = reader->read_image();
//             XRSLAMImage image;
//             image.camera_id = 0;
//             image.timeStamp = t;
//             image.ext = nullptr;
//             image.data = img.data;
//             image.stride = img.step[0];
//             std::cout << "image: t" << std::to_string(t) << std::endl;
//             XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
//             if (has_accelerometer && has_gyroscope) {
//                 XRSLAMRunOneFrame();
//                 XRSLAMState state;
//                 XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
//                 if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
// #ifndef XRSLAM_PC_HEADLESS_ONLY
//                     int cols = 0, rows = 0;
//                     reader->get_image_resolution(cols, rows);
//                     cv::Mat img = cv::Mat(rows, cols, CV_8UC1, image.data);
//                     cv::cvtColor(img, cur_frame_viz_.show_frame,
//                                  cv::COLOR_GRAY2BGR);
//                     GetShowElements(cur_frame_viz_);
//                     visualizer.update_frame(
//                         std::make_shared<VizElements>(cur_frame_viz_));
// #endif
//                     XRSLAMPose pose_b;
//                     XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
//                     if (pose_b.timestamp > 0) {
//                         for (auto &output : outputs) {
//                             output->write_pose(pose_b.timestamp, pose_b);
//                         }
//                     }
//                 }
//             }
//         } break;
//         default: {
//         } break;
//         }
//     }
// #ifndef XRSLAM_PC_HEADLESS_ONLY
//     if (visualizer_thread_.joinable()) {
//         visualizer_thread_.join();
//         std::cout << "\ndestory visualizer thread ..." << std::endl;
//     }
// #endif
//     XRSLAMDestroy();

//     return EXIT_SUCCESS;
}

// error
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
