#include <xrslam/estimation/reprojection_factor.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/poisson_disk_filter.h>

namespace xrslam {

struct Frame::construct_by_frame_t {};

Frame::Frame() : map(nullptr) {}

Frame::Frame(const Frame &frame, const construct_by_frame_t &construct_by_frame)
    : Tagged(frame), Identifiable(frame), map(nullptr) {}

Frame::~Frame() = default;

std::unique_ptr<Frame> Frame::clone() const {
    std::unique_ptr<Frame> frame =
        std::make_unique<Frame>(*this, construct_by_frame_t());
    frame->K = K;
    frame->sqrt_inv_cov = sqrt_inv_cov;
    frame->image = image;
    frame->pose = pose;
    frame->motion = motion;
    frame->camera = camera;
    frame->imu = imu;
    frame->preintegration = preintegration;
    frame->bearings = bearings;
    frame->tracks = std::vector<Track *>(bearings.size(), nullptr);
    frame->reprojection_error_factors =
        std::vector<std::unique_ptr<ReprojectionErrorFactor>>(bearings.size());
    frame->map = nullptr;
    return frame;
}

void Frame::append_keypoint(const vector<3> &keypoint) {
    bearings.emplace_back(keypoint);
    tracks.emplace_back(nullptr);
    reprojection_error_factors.emplace_back(nullptr);
}

Track *Frame::get_track(size_t keypoint_index, Map *allocation_map) {
    if (!allocation_map) {
        allocation_map = map;
    }
    if (tracks[keypoint_index] == nullptr) {
        Track *track = allocation_map->create_track();
        track->add_keypoint(this, keypoint_index);
    }
    return tracks[keypoint_index];
}
// 通过在图像中检测新特征点来更新帧的关键点信息，并准备相应的数据结构
void Frame::detect_keypoints(Config *config) {
    // 初始化现有关键点像素坐标
    std::vector<vector<2>> pkeypoints(bearings.size());
    for (size_t i = 0; i < bearings.size(); ++i) {
        pkeypoints[i] = apply_k(bearings[i], K);
    }
    // 根据配置文件中的参数，使用特征点检测算法在图像中检测新的特征点
    image->detect_keypoints(pkeypoints,
                            config->feature_tracker_max_keypoint_detection(),
                            config->feature_tracker_min_keypoint_distance());
    // 更新数据结构，以适应新的特征点数量
    // old_keypoint_num 用于区分已有关键点和新增关键点
    size_t old_keypoint_num = bearings.size();
    // 扩展 bearings：存储新增关键点的三维方向向量
    bearings.resize(pkeypoints.size());
    // 扩展 tracks：存储新增关键点的跟踪状态
    tracks.resize(pkeypoints.size(), nullptr);
    // 扩展 reprojection_error_factors：存储新增关键点的重投影误差因子
    reprojection_error_factors.resize(pkeypoints.size());
    // 将新增关键点的三维方向向量计算出来
    for (size_t i = old_keypoint_num; i < pkeypoints.size(); ++i) {
        // remove_k 是 apply_k 的逆操作，通过相机内参矩阵 K 恢复方向向量
        bearings[i] = remove_k(pkeypoints[i], K);
    }
}

// 实现视觉前端的特征点跟踪功能
void Frame::track_keypoints(Frame *next_frame, Config *config) {
    // 当前帧特征点初始化 bearings是当前帧地图点？
    std::vector<vector<2>> curr_keypoints(bearings.size());
    std::vector<vector<2>> next_keypoints;
    // apply_k 使用相机内参矩阵 K 将特征点从归一化平面投影到像素平面
    for (size_t i = 0; i < bearings.size(); ++i) {
        curr_keypoints[i] = apply_k(bearings[i], K);
    }
    // 如果配置文件中启用了特征点预测，则根据当前帧和下一帧的姿态变化预测下一帧的特征点位置
    if (config->feature_tracker_predict_keypoints()) {

        // 使用 IMU 数据的旋转增量 delta.q 预测下一帧中特征点的方向
        quaternion delta_key_q =
            (camera.q_cs.conjugate() * imu.q_cs *
             next_frame->preintegration.delta.q *
             next_frame->imu.q_cs.conjugate() * next_frame->camera.q_cs)
                .conjugate();
        //  重置 next_keypoints       
        next_keypoints.resize(curr_keypoints.size());
        // apply_k 使用相机内参矩阵 K 将特征点从归一化平面投影到像素平面
        for (size_t i = 0; i < bearings.size(); ++i) {
            // 将特征点方向旋转到下一帧的方向
            next_keypoints[i] =
                apply_k(delta_key_q * bearings[i], next_frame->K);
        }
    }

    // 使用特征点跟踪器跟踪特征点，并返回跟踪状态和掩码
    std::vector<char> status, mask;
    // 光流跟踪，返回特征点在下一帧中的位置 next_keypoints
    image->track_keypoints(next_frame->image.get(), curr_keypoints,
                           next_keypoints, status);
    // 将当前帧和下一帧的特征点归一化到相机坐标系中
    std::vector<vector<2>> curr_keypoints_h, next_keypoints_h;
    std::vector<vector<3>> next_bearings;
    // 遍历当前帧特征点
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        // 将当前帧的特征点单位化（去掉尺度，转化为方向矢量）
        curr_keypoints_h.push_back(bearings[i].hnormalized());
        // 下一帧特征点转化为相机坐标系
        vector<3> next_keypoint = remove_k(next_keypoints[i], next_frame->K);
        // 将下一帧特征点单位化（去掉尺度，转化为方向矢量）
        next_keypoints_h.push_back(next_keypoint.hnormalized());
        // 将下一帧特征点方向向量存储到 next_bearings 中
        next_bearings.push_back(next_keypoint);
    }
    // 利用匹配点求解两帧间的本质矩阵，剔除不一致的点
    matrix<3> E =
        find_essential_matrix(curr_keypoints_h, next_keypoints_h, mask, 1.0);
    //  剔除错误匹配点
    for (size_t i = 0; i < status.size(); ++i) {
        if (!mask[i]) {
            status[i] = 0;
        }
    }
    // 计算两帧间的旋转矩阵
    matrix<3> R = find_rotation_matrix(bearings, next_bearings, mask,
                                       (M_PI / 180.0) *
                                           config->rotation_ransac_threshold());
    // 计算旋转后的方向向量与匹配向量之间的角度误差，用于进一步分析
    std::vector<double> angles;
    for (size_t i = 0; i < mask.size(); ++i) {
        if (mask[i]) {
            // 1. 对每个有效点（mask[i] == true），计算旋转后的向量 R * bearings[i]。
            // 2. 计算旋转后的向量与对应的下一帧方向向量 next_bearings[i] 的点积。
            // 3. 使用 acos 计算角度误差，并将其转化为角度制（乘以 180/π）。
            // 4. 将结果保存到 angles 中
            double angle = acos((R * bearings[i]).dot(next_bearings[i]));
            angles.emplace_back(angle * 180 / M_PI);
        }
    }
    // 将 angles 按从小到大排序
    std::sort(angles.begin(), angles.end());
    // 如果 angles 不为空，选择第 70% 分位点的值作为错位
    double misalignment =
        angles.size() > 0 ? angles[angles.size() * 7 / 10] : 0;
    // 使用 inspect_debug 保存错位数据
    inspect_debug(feature_tracker_angle_misalignment, angle_misalignment) {
        angle_misalignment = misalignment;
    }
    // 如果错位小于配置的阈值 (rotation_misalignment_threshold)，表示旋转较小
    if (misalignment < config->rotation_misalignment_threshold()) {
        // 可能没有明显平移，设置帧标签 FT_NO_TRANSLATION
        next_frame->tag(FT_NO_TRANSLATION) = true;
    }

    // filter keypoints based on track length
    // 根据轨迹长度过滤特征点
    // 定义一个存储特征点索引和轨迹长度的vector
    std::vector<std::pair<size_t, size_t>> keypoint_index_track_length;
    keypoint_index_track_length.reserve(curr_keypoints.size());
    // 遍历当前帧的特征点
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        // 如果 status[i] == 0，表示该关键点未被成功跟踪，跳过
        if (status[i] == 0)
            continue;
        // 调用 get_track(i) 获取第 i 个关键点的轨迹对象    
        Track *track = get_track(i);
        // 如果轨迹对象为空，表示该关键点没有对应的轨迹，跳过
        if (track == nullptr)
            continue;
        // 保存关键点索引和轨迹长度    
        keypoint_index_track_length.emplace_back(i, track->keypoint_num());
    }

    // 按轨迹长度排序关键点
    std::sort(keypoint_index_track_length.begin(),
              keypoint_index_track_length.end(),
              [](const auto &a, const auto &b) { return a.second > b.second; });
    // 创建一个 PoissonDiskFilter 对象，用于限制关键点之间的最小距离，避免密集分布的关键点降低跟踪性能
    PoissonDiskFilter<2> filter(
        config->feature_tracker_min_keypoint_distance());
    //  keypoint_index_track_length 获取关键点索引和对应的轨迹长度
    for (auto &[keypoint_index, track_length] : keypoint_index_track_length) {
        // 获取关键点的二维坐标 pt
        vector<2> pt = next_keypoints[keypoint_index];
        // 获取该关键点的轨迹对象 track
        Track *track = this->get_track(keypoint_index);
        // 1. 关键点是否满足最小距离要求
        // 2. 关键点对应的轨迹是否有效，若轨迹标记为垃圾（TT_TRASH），则不使用。
        if (filter.permit_point(pt) &&
            (!track || (track && !track->tag(TT_TRASH)))) {
            // 满足条件的关键点被加入 filter 中    
            filter.preset_point(pt);
        } else {
            // 不满足条件的关键点（被过滤掉的）标记为 status[keypoint_index] = 0
            status[keypoint_index] = 0;
        }
    }
    // 遍历当前帧的关键点
    for (size_t curr_keypoint_index = 0;
         curr_keypoint_index < curr_keypoints.size(); ++curr_keypoint_index) {
        // 检查 status[curr_keypoint_index] 是否为有效状态（1 表示有效）    
        if (status[curr_keypoint_index]) {
            // 获取下一帧中即将分配的关键点索引 next_keypoint_index
            size_t next_keypoint_index = next_frame->keypoint_num();
            // 将当前帧中关键点对应的方向（bearing）添加到下一帧中
            next_frame->append_keypoint(next_bearings[curr_keypoint_index]);
            // 使用 add_keypoint 更新轨迹，关联当前帧和下一帧的关键点
            get_track(curr_keypoint_index, nullptr)
                ->add_keypoint(next_frame, next_keypoint_index);
        }
    }
}

PoseState Frame::get_pose(const ExtrinsicParams &sensor) const {
    PoseState result;
    result.q = pose.q * sensor.q_cs;
    result.p = pose.p + pose.q * sensor.p_cs;
    return result;
}

void Frame::set_pose(const ExtrinsicParams &sensor, const PoseState &pose) {
    this->pose.q = pose.q * sensor.q_cs.conjugate();
    this->pose.p = pose.p - this->pose.q * sensor.p_cs;
}

std::unique_lock<std::mutex> Frame::lock() const {
    if (map) {
        return map->lock();
    } else {
        return {};
    }
}

} // namespace xrslam
