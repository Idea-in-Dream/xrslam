#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/initializer.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/essential.h>
#include <xrslam/geometry/homography.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/xrslam.h>

namespace xrslam {

Initializer::Initializer(std::shared_ptr<Config> config) : config(config) {}

Initializer::~Initializer() = default;

// 从 feature_tracking_map 中提取一定数量的关键帧，克隆到一个新的地图 map 中，
// 并重新建立关键帧之间的轨迹和预积分数据。
void Initializer::mirror_keyframe_map(Map *feature_tracking_map,
                                      size_t init_frame_id) {
    // 根据 init_frame_id 获取其在 feature_tracking_map 中的索引位置
    size_t init_frame_index_last =
        feature_tracking_map->frame_index_by_id(init_frame_id);
    // 关键帧之间的间隔，取值来自配置    
    size_t init_frame_index_gap = config->initializer_keyframe_gap();
    // 所有初始化帧要求的总数量，取值来自配置
    size_t init_frame_index_distance =
        init_frame_index_gap * (config->initializer_keyframe_num() - 1);
    // init_frame_id置为空，等待下一次函数调用赋值 
    init_frame_id = nil();
    // 如果 feature_tracking_map 中的帧数量不足，无法满足初始化要求，则清空 map 并返回
    if (init_frame_index_last < init_frame_index_distance) {
        map.reset();
        return;
    }
    // 计算第一个初始化帧的索引位置
    size_t init_frame_index_first =
        init_frame_index_last - init_frame_index_distance;
    // 计算初始化关键帧的索引集合 (1,6,11,...)
    std::vector<size_t> init_keyframe_indices;
    for (size_t i = 0; i < config->initializer_keyframe_num(); ++i) {
        init_keyframe_indices.push_back(init_frame_index_first +
                                        i * init_frame_index_gap);
    }
    // 创建一个新的地图 map
    map = std::make_unique<Map>();
    // 从 feature_tracking_map 中克隆关键帧，并将其附加到 map 中
    for (size_t index : init_keyframe_indices) {
        map->attach_frame(feature_tracking_map->get_frame(index)->clone());
    }

    //遍历 map 中的关键帧，从第 1 个关键帧开始，依次处理前后相邻的关键帧对
    for (size_t j = 1; j < map->frame_num(); ++j) {
        // 原始地图中对应的关键帧
        Frame *old_frame_i =
            feature_tracking_map->get_frame(init_keyframe_indices[j - 1]);
        Frame *old_frame_j =
            feature_tracking_map->get_frame(init_keyframe_indices[j]);
        // 新地图中克隆的对应关键帧
        Frame *new_frame_i = map->get_frame(j - 1);
        Frame *new_frame_j = map->get_frame(j);
        
        // 遍历 old_frame_i 中的所有关键点
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            // 获取每个关键点对应的 Track 对象
            if (Track *track = old_frame_i->get_track(ki)) {
                // 如果关键点 ki 在 old_frame_i 和 old_frame_j 之间有轨迹连接
                // （get_keypoint_index 返回有效的 kj）
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    // 在 new_frame_i 和 new_frame_j 中对应的关键点重新建立轨迹连接        
                    new_frame_i->get_track(ki, nullptr)
                        ->add_keypoint(new_frame_j, kj);
                }
            }
        }
        // 清除新帧的 IMU 数据：
        new_frame_j->preintegration.data.clear();
        // 遍历 feature_tracking_map 中的帧，逐帧将 preintegration.data 中的 IMU 数据从 old_frame 复制到 new_frame_j 中
        for (size_t f = init_keyframe_indices[j - 1];
             f < init_keyframe_indices[j]; ++f) {
            // 原始地图中对应的关键帧    
            Frame *old_frame = feature_tracking_map->get_frame(f + 1);
            // 关键帧对应的IMU预积分
            std::vector<ImuData> &old_data = old_frame->preintegration.data;
            // 将 old_data 中的 IMU 数据复制到 new_frame_j 中
            std::vector<ImuData> &new_data = new_frame_j->preintegration.data;
            // 依次往后插入 old_data 中的元素到 new_data 中
            new_data.insert(new_data.end(), old_data.begin(), old_data.end());
        }
    }
}

// SLAM初始化器的核心部分，通过构建地图、初始化IMU和三角化特征点，完成对位姿和地图的优化，并返回一个用于后续跟踪的 SlidingWindowTracker 对象
std::unique_ptr<SlidingWindowTracker> Initializer::initialize() {
    // 检查地图是否为空：如果 map 为空，直接返回 nullptr
    if (!map)
        return nullptr;
    // 是否完成sfm的初始化：如果 init_sfm() 返回 false，直接返回 nullptr
    if (!init_sfm())
        return nullptr;
    // 是否完成IMU的初始化：如果 init_imu() 返回 false，直接返回 nullptr
    if (!init_imu())
        return nullptr;

    // 固定第一个关键帧的位姿,作为初始化阶段的参考帧，避免全局漂移
    map->get_frame(0)->tag(FT_FIX_POSE) = true;
    // 创建一个非线性优化求解器 solver
    auto solver = Solver::create();
    // 将所有关键帧的位姿状态加入到求解器中，供后续优化使用
    for (size_t i = 0; i < map->frame_num(); ++i) {
        solver->add_frame_states(map->get_frame(i));
    }
    // 被访问的轨迹状态
    std::unordered_set<Track *> visited_tracks;
    // 遍历所有的关键帧
    for (size_t i = 0; i < map->frame_num(); ++i) {
        // 获取关键帧
        Frame *frame = map->get_frame(i);
        // 遍历关键帧中的所有关键点
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            // 获取关键点对应的轨迹
            Track *track = frame->get_track(j);
            // 如果轨迹为空，跳过
            if (!track)
                continue;
            // 如果未标记为 TT_VALID），跳过    
            if (!track->tag(TT_VALID))
                continue;
            // 如果已经被访问添加过了    
            if (visited_tracks.count(track) > 0)
                continue;
            // 使用 visited_tracks 避免重复添加轨迹    
            visited_tracks.insert(track);
            // 将轨迹状态（如 3D 点位置）加入到求解器
            solver->add_track_states(track);
        }
    }
    // 遍历所有的关键帧
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        // 遍历关键帧中的所有关键点
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            // 获取关键点对应的轨迹
            Track *track = frame->get_track(j);
            // 如果轨迹为空，跳过
            if (!track)
                continue;
            // 如果如果未标记为 TT_VALID，或者 TT_TRIANGULATED    
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED))
                continue;
            // 如果是首帧    
            if (frame == track->first_frame())
                continue;
            // 添加重投影误差因子    
            solver->add_factor(frame->reprojection_error_factors[j].get());
        }
    }
    // 遍历所有的相邻关键帧对
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        // 相邻关键帧进行预积分，预积分成功，则添加预积分误差因子
        if (frame_j->preintegration.integrate(frame_j->image->t,
                                              frame_i->motion.bg,
                                              frame_i->motion.ba, true, true)) {
            //  创造预积分误差因子并添加                       
            solver->put_factor(Solver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->preintegration));
        }
    }
    // 执行优化，调整所有关键帧的位姿和地图点的位置
    solver->solve();

    // 将所有帧标记为关键帧，用于后续的滑动窗口优化
    for (size_t i = 0; i < map->frame_num(); ++i) {
        map->get_frame(i)->tag(FT_KEYFRAME) = true;
    }

    // 调试与可视化
    inspect_debug(sliding_window_landmarks, landmarks) {
        // 地图点集合
        std::vector<Landmark> points;
        points.reserve(map->track_num());
        // 遍历所有地图点轨迹
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                // 如果轨迹被标记为 TT_VALID
                if (track->tag(TT_VALID)) {
                    // 创建地图点
                    Landmark point;
                    // 得到地图点
                    point.p = track->get_landmark_point();
                    // 标记为已三角化
                    point.triangulated = track->tag(TT_TRIANGULATED);
                    // 添加到集合中
                    points.push_back(point);
                }
            }
        }
        // 将地图点集合赋值给 landmarks
        landmarks = std::move(points);
    }

    // 创建并返回一个 SlidingWindowTracker 对象
    // 将优化后的 map 传递给其进行后续跟踪
    std::unique_ptr<SlidingWindowTracker> tracker =
        std::make_unique<SlidingWindowTracker>(std::move(map), config);
    return tracker;
}

bool Initializer::init_sfm() {
    // [1] try initializing using raw_map
    Frame *init_frame_i = map->get_frame(0);
    Frame *init_frame_j = map->get_frame(map->frame_num() - 1);

    double total_parallax = 0;
    int common_track_num = 0;
    std::vector<vector<3>> init_points;
    std::vector<std::pair<size_t, size_t>> init_matches;
    std::vector<char> init_point_status;
    std::vector<vector<2>> frame_i_keypoints;
    std::vector<vector<2>> frame_j_keypoints;
    matrix<3> init_R;
    vector<3> init_T;

    for (size_t ki = 0; ki < init_frame_i->keypoint_num(); ++ki) {
        Track *track = init_frame_i->get_track(ki);
        if (!track)
            continue;
        size_t kj = track->get_keypoint_index(init_frame_j);
        if (kj == nil())
            continue;
        frame_i_keypoints.push_back(
            init_frame_i->get_keypoint(ki).hnormalized());
        frame_j_keypoints.push_back(
            init_frame_j->get_keypoint(kj).hnormalized());
        init_matches.emplace_back(ki, kj);
        total_parallax +=
            (apply_k(init_frame_i->get_keypoint(ki), init_frame_i->K) -
             apply_k(init_frame_j->get_keypoint(kj), init_frame_j->K))
                .norm();
        common_track_num++;
    }

    if (common_track_num < (int)config->initializer_min_matches())
        return false;
    total_parallax /= std::max(common_track_num, 1);
    if (total_parallax < config->initializer_min_parallax())
        return false;

    std::vector<matrix<3>> Rs;
    std::vector<vector<3>> Ts;

    matrix<3> RH1, RH2;
    vector<3> TH1, TH2, nH1, nH2;
    matrix<3> H = find_homography_matrix(frame_i_keypoints, frame_j_keypoints,
                                         0.7 / init_frame_i->K(0, 0), 0.999,
                                         1000, config->random());
    if (!decompose_homography(H, RH1, RH2, TH1, TH2, nH1, nH2)) {
        log_debug("SfM init fail: pure rotation.");
        return false; // is pure rotation
    }
    TH1 = TH1.normalized();
    TH2 = TH2.normalized();
    Rs.insert(Rs.end(), {RH1, RH1, RH2, RH2});
    Ts.insert(Ts.end(), {TH1, -TH1, TH2, -TH2});

    matrix<3> RE1, RE2;
    vector<3> TE;
    matrix<3> E = find_essential_matrix(frame_i_keypoints, frame_j_keypoints,
                                        0.7 / init_frame_i->K(0, 0), 0.999,
                                        1000, config->random());
    decompose_essential(E, RE1, RE2, TE);
    TE = TE.normalized();
    Rs.insert(Rs.end(), {RE1, RE1, RE2, RE2});
    Ts.insert(Ts.end(), {TE, -TE, TE, -TE});

    // [1.1] triangulation
    std::vector<std::vector<vector<3>>> triangulation_points(Rs.size());
    std::vector<std::vector<char>> triangulation_status(Rs.size());
    std::vector<size_t> triangulation_counts(Rs.size());
    std::vector<double> triangulation_scores(Rs.size());

    size_t best_rt_index = 0;
    for (size_t i = 0; i < Rs.size(); ++i) {
        auto &points = triangulation_points[i];
        auto &status = triangulation_status[i];
        auto &count = triangulation_counts[i];
        auto &score = triangulation_scores[i];
        points.resize(frame_i_keypoints.size());
        status.resize(frame_i_keypoints.size());
        count = 0;
        score = 0;
        matrix<3, 4> P1, P2;
        P1.setIdentity();
        P2 << Rs[i], Ts[i];
        for (size_t j = 0; j < frame_i_keypoints.size(); ++j) {
            status[j] = 0;

            vector<4> q =
                triangulate_point(P1, P2, frame_i_keypoints[j].homogeneous(),
                                  frame_j_keypoints[j].homogeneous());
            vector<3> q1 = P1 * q;
            vector<3> q2 = P2 * q;
            if (q1[2] * q[3] > 0 && q2[2] * q[3] > 0) {
                if (q1[2] / q[3] < 100 && q2[2] / q[3] < 100) {
                    points[j] = q.hnormalized();
                    status[j] = 1;
                    count++;
                    score += 0.5 * ((q1.hnormalized() - frame_i_keypoints[j])
                                        .squaredNorm() +
                                    (q2.hnormalized() - frame_j_keypoints[j])
                                        .squaredNorm());
                }
            }
        }

        if (triangulation_counts[i] > config->initializer_min_triangulation() &&
            triangulation_scores[i] < triangulation_scores[best_rt_index]) {
            best_rt_index = i;
        } else if (triangulation_counts[i] >
                   triangulation_counts[best_rt_index]) {
            best_rt_index = i;
        }
    }
    init_R = Rs[best_rt_index];
    init_T = Ts[best_rt_index];
    init_points.swap(triangulation_points[best_rt_index]);
    init_point_status.swap(triangulation_status[best_rt_index]);
    size_t triangulated_num = triangulation_counts[best_rt_index];

    if (triangulated_num < config->initializer_min_triangulation()) {
        log_debug("SfM init fail: triangulation (%zd).", triangulated_num);
        return false;
    }

    // [2] create sfm map

    // [2.1] set init states
    PoseState pose;
    pose.q.setIdentity();
    pose.p.setZero();
    init_frame_i->set_pose(init_frame_i->camera, pose);
    pose.q = init_R.transpose();
    pose.p = -(init_R.transpose() * init_T);
    init_frame_j->set_pose(init_frame_j->camera, pose);

    for (size_t k = 0; k < init_points.size(); ++k) {
        if (init_point_status[k] == 0)
            continue;
        Track *track = init_frame_i->get_track(init_matches[k].first);
        track->set_landmark_point(init_points[k]);
        track->tag(TT_VALID) = true;
        track->tag(TT_TRIANGULATED) = true;
    }

    // [2.2] solve other frames via pnp
    for (size_t j = 1; j + 1 < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->set_pose(frame_j->camera, frame_i->get_pose(frame_i->camera));
        auto solver = Solver::create();
        solver->add_frame_states(frame_j);
        for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
            Track *track = frame_j->get_track(k);
            if (!track)
                continue;
            if (!track->has_keypoint(map->get_frame(0)))
                continue;
            if (track->tag(TT_VALID) && track->tag(TT_TRIANGULATED)) {
                solver->put_factor(
                    Solver::create_reprojection_prior_factor(frame_j, track));
            }
        }
        solver->solve();
    }

    // [2.3] triangulate more points
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (track->tag(TT_VALID))
            continue;
        if (auto p = track->triangulate()) {
            track->set_landmark_point(p.value());
            track->tag(TT_VALID) = true;
            track->tag(TT_TRIANGULATED) = true;
        }
    }

    // [3] sfm

    // [3.1] bundle adjustment
    map->get_frame(0)->tag(FT_FIX_POSE) = true;
    auto solver = Solver::create();
    for (size_t i = 0; i < map->frame_num(); ++i) {
        solver->add_frame_states(map->get_frame(i), false);
    }
    std::unordered_set<Track *> visited_tracks;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->tag(TT_VALID))
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            solver->add_track_states(track);
        }
    }
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED))
                continue;
            if (frame == track->first_frame())
                continue;
            solver->add_factor(frame->reprojection_error_factors[j].get());
        }
    }
    if (!solver->solve()) {
        return false;
    }

    // [3.2] cleanup invalid points
    map->prune_tracks([](const Track *track) {
        return !track->tag(TT_VALID) || track->landmark.reprojection_error >
                                            3.0; // TODO: make configurable
    });

    return true;
}

bool Initializer::init_imu() {
    reset_states();
    solve_gyro_bias();
    solve_gravity_scale_velocity();
    if (scale < 0.001 || scale > 1.0)
        return false;
    if (!config->initializer_refine_imu()) {
        return apply_init();
    }
    refine_scale_velocity_via_gravity();
    if (scale < 0.001 || scale > 1.0)
        return false;
    return apply_init();
}

void Initializer::solve_gyro_bias() {
    preintegrate();
    matrix<3> A = matrix<3>::Zero();
    vector<3> b = vector<3>::Zero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);

        const PoseState pose_i = frame_i->get_pose(frame_i->imu);
        const PoseState pose_j = frame_j->get_pose(frame_j->imu);

        const quaternion &dq = frame_j->preintegration.delta.q;
        const matrix<3> &dq_dbg = frame_j->preintegration.jacobian.dq_dbg;
        A += dq_dbg.transpose() * dq_dbg;
        b +=
            dq_dbg.transpose() * logmap((pose_i.q * dq).conjugate() * pose_j.q);
    }

    Eigen::JacobiSVD<matrix<3>> svd(A,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
    bg = svd.solve(b);
}

void Initializer::solve_gravity_scale_velocity() {
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    A.resize((N - 1) * 6, 3 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    A.setZero();
    b.setZero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);
        const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
        const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
        const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

        A.block<3, 3>(i * 6, 0) =
            -0.5 * delta.t * delta.t * matrix<3>::Identity();
        A.block<3, 1>(i * 6, 3) = camera_pose_j.p - camera_pose_i.p;
        A.block<3, 3>(i * 6, 4 + i * 3) = -delta.t * matrix<3>::Identity();
        b.segment<3>(i * 6) = frame_i->pose.q * delta.p +
                              (frame_j->pose.q * frame_j->camera.p_cs -
                               frame_i->pose.q * frame_i->camera.p_cs);

        A.block<3, 3>(i * 6 + 3, 0) = -delta.t * matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + i * 3) = -matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + j * 3) = matrix<3>::Identity();
        b.segment<3>(i * 6 + 3) = frame_i->pose.q * delta.v;
    }

    vector<> x = A.fullPivHouseholderQr().solve(b);
    gravity = x.segment<3>(0).normalized() * XRSLAM_GRAVITY_NOMINAL;
    scale = x(3);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(4 + i * 3);
    }
}

void Initializer::refine_scale_velocity_via_gravity() {
    static const double damp = 0.1;
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    vector<> x;
    A.resize((N - 1) * 6, 2 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    x.resize(2 + 1 + 3 * N);

    for (size_t iter = 0; iter < 1; ++iter) {
        A.setZero();
        b.setZero();
        matrix<3, 2> Tg = s2_tangential_basis(gravity);

        for (size_t j = 1; j < map->frame_num(); ++j) {
            const size_t i = j - 1;

            const Frame *frame_i = map->get_frame(i);
            const Frame *frame_j = map->get_frame(j);
            const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
            const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
            const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

            A.block<3, 2>(i * 6, 0) = -0.5 * delta.t * delta.t * Tg;
            A.block<3, 1>(i * 6, 2) = camera_pose_j.p - camera_pose_i.p;
            A.block<3, 3>(i * 6, 3 + i * 3) = -delta.t * matrix<3>::Identity();
            b.segment<3>(i * 6) = 0.5 * delta.t * delta.t * gravity +
                                  frame_i->pose.q * delta.p +
                                  (frame_j->pose.q * frame_j->camera.p_cs -
                                   frame_i->pose.q * frame_i->camera.p_cs);

            A.block<3, 2>(i * 6 + 3, 0) = -delta.t * Tg;
            A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -matrix<3>::Identity();
            A.block<3, 3>(i * 6 + 3, 3 + j * 3) = matrix<3>::Identity();
            b.segment<3>(i * 6 + 3) =
                delta.t * gravity + frame_i->pose.q * delta.v;
        }

        x = A.fullPivHouseholderQr().solve(b);
        vector<2> dg = x.segment<2>(0);
        gravity =
            (gravity + damp * Tg * dg).normalized() * XRSLAM_GRAVITY_NOMINAL;
    }

    scale = x(2);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(3 + i * 3);
    }
}

void Initializer::reset_states() {
    bg.setZero();
    ba.setZero();
    gravity.setZero();
    scale = 1;
    velocities.resize(map->frame_num(), vector<3>::Zero());
}

void Initializer::preintegrate() {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, bg, ba, true,
                                          false);
    }
}

bool Initializer::apply_init(bool apply_ba, bool apply_velocity) {
    static const vector<3> gravity_nominal{0, 0, -XRSLAM_GRAVITY_NOMINAL};

    quaternion q = quaternion::FromTwoVectors(gravity, gravity_nominal);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        PoseState imu_pose = frame->get_pose(frame->imu);
        imu_pose.q = q * imu_pose.q;
        imu_pose.p = scale * (q * imu_pose.p);
        frame->set_pose(frame->imu, imu_pose);
        if (apply_velocity) {
            frame->motion.v = q * velocities[i];
        } else {
            frame->motion.v.setZero();
        }
        frame->motion.bg = bg;
        if (apply_ba) {
            frame->motion.ba = ba;
        } else {
            frame->motion.ba.setZero();
        }
    }
    size_t final_point_num = 0;
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (auto p = track->triangulate()) {
            track->set_landmark_point(p.value());
            track->tag(TT_VALID) = true;
            track->tag(TT_TRIANGULATED) = true;
            final_point_num++;
        } else {
            track->tag(TT_VALID) = false;
        }
    }

    return final_point_num >= config->initializer_min_landmarks();
}

} // namespace xrslam
