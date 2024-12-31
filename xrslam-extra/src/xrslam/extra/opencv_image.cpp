#include <xrslam/extra/opencv_image.h>
#include <xrslam/extra/poisson_disk_filter.h>

using namespace cv;

namespace xrslam::extra {


static std::vector<Point2f> to_opencv(const std::vector<vector<2>> &v) {
    std::vector<Point2f> r(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        r[i].x = (float)v[i].x();
        r[i].y = (float)v[i].y();
    }
    return r;
}

OpenCvImage::OpenCvImage() = default;

double OpenCvImage::evaluate(const vector<2> &u, int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f);
    return f;
}

double OpenCvImage::evaluate(const vector<2> &u, vector<2> &ddu,
                             int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f, &ddu.y(), &ddu.x());
    ddu.x() *= s.x();
    ddu.y() *= s.y();
    return f;
}

void OpenCvImage::detect_keypoints(std::vector<vector<2>> &keypoints,
                                   size_t max_points,
                                   double keypoint_distance) const {

    std::vector<KeyPoint> cvkeypoints;

    gftt(max_points)->detect(image, cvkeypoints);

    if (cvkeypoints.size() > 0) {
        std::sort(cvkeypoints.begin(), cvkeypoints.end(),
                  [](const auto &a, const auto &b) {
                      return a.response > b.response;
                  });
        std::vector<vector<2>> new_keypoints;
        for (size_t i = 0; i < cvkeypoints.size(); ++i) {
            new_keypoints.emplace_back(cvkeypoints[i].pt.x,
                                       cvkeypoints[i].pt.y);
        }

        PoissonDiskFilter<2> filter(keypoint_distance);
        filter.preset_points(keypoints);
        filter.insert_points(new_keypoints);

        new_keypoints.erase(
            std::remove_if(new_keypoints.begin(), new_keypoints.end(),
                           [this](const auto &keypoint) {
                               return keypoint.x() < 20 || keypoint.y() < 20 ||
                                      keypoint.x() >= image.cols - 20 ||
                                      keypoint.y() >= image.rows - 20;
                           }),
            new_keypoints.end());

        keypoints.insert(keypoints.end(), new_keypoints.begin(),
                         new_keypoints.end());
    }
}

void OpenCvImage::track_keypoints(const Image *next_image,
                                  const std::vector<vector<2>> &curr_keypoints,
                                  std::vector<vector<2>> &next_keypoints,
                                  std::vector<char> &result_status) const {
    // 关键点转换成opencv的cv::Point2f格式
    std::vector<Point2f> curr_cvpoints = to_opencv(curr_keypoints);
    // 定义预测的下一幅图像的关键点
    std::vector<Point2f> next_cvpoints;

    // 如果有预测的关键点，进行关键点格式转换
    if (next_keypoints.size() > 0) {
        next_cvpoints = to_opencv(next_keypoints);
    } else {
        // 如果没有预测的关键点，则使用当前关键点作为预测
        next_keypoints.resize(curr_keypoints.size());
        next_cvpoints = curr_cvpoints;
    }
    // 获取下一幅图像 针 next_image 动态转换为 OpenCvImage 类型的指针，并将结果赋值给 next_cvimage
    const OpenCvImage *next_cvimage =
        dynamic_cast<const OpenCvImage *>(next_image);
    // 定义跟踪状态 Vector   result_status ，大小为当前关键点的大小，初始值为0
    result_status.resize(curr_keypoints.size(), 0);
    // 如果有下一幅图像，并且当前关键点不为空，则进行光流跟踪
    if (next_cvimage && curr_cvpoints.size() > 0) {
        Mat cvstatus, cverr;
        // 使用calcOpticalFlowPyrLK函数进行光流跟踪
        calcOpticalFlowPyrLK(
            image_pyramid, next_cvimage->image_pyramid, curr_cvpoints,
            next_cvpoints, cvstatus, cverr, Size(21, 21), (int)level_num(),
            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
            OPTFLOW_USE_INITIAL_FLOW);
        // 判断点是否接近图像边缘（距离小于 20 像素）
        for (size_t i = 0; i < next_cvpoints.size(); ++i) {
            result_status[i] = cvstatus.at<unsigned char>((int)i);
            if (next_cvpoints[i].x < 20 ||
                next_cvpoints[i].x >= image.cols - 20 ||
                next_cvpoints[i].y < 20 ||
                next_cvpoints[i].y >= image.rows - 20) {
                result_status[i] = 0;
            }
            // 如果初步状态是有效的
            if (result_status[i]) {
                // 计算位移向量
                auto p = (next_cvpoints[i] - curr_cvpoints[i]);
                // 转换为 2D 向量
                auto v = vector<2>(p.x, p.y);
                // 如果位移向量大于图像的四分之一长度
                if (v.norm() > image.rows / 4) {
                    // 标记为无效 
                    result_status[i] = 0;
                }
            }
        }
        // 反向光流跟踪
        {   
            // 反向光流状态量
            std::vector<uchar> reverse_status;
            // 反向光流误差
            std::vector<float> reverse_err;
            // 反向光流点，初始为当前帧光流点
            std::vector<Point2f> reverse_pts = curr_cvpoints;
            // 反向光流跟踪
            calcOpticalFlowPyrLK(
                next_cvimage->image_pyramid, image_pyramid, next_cvpoints,
                reverse_pts, reverse_status, reverse_err, Size(21, 21),
                (int)level_num(),
                TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
                OPTFLOW_USE_INITIAL_FLOW);

            for (size_t i = 0; i < reverse_status.size(); ++i) {
                // 正向跟踪的状态为成功
                if (result_status[i]) {
                    // 反向跟踪失败 或者 反向光流点与正向光流点距离大于 0.5 像素
                    if (!reverse_status[i] ||
                        cv::norm(curr_cvpoints[i] - reverse_pts[i]) > 0.5) {
                        // 标记为无效
                        result_status[i] = 0;
                    }
                }
            }
        }
    }

    // std::vector<size_t> l;
    // std::vector<Point2f> p, q;
    // for (size_t i = 0; i < result_status.size(); ++i) {
    //     if (result_status[i] != 0) {
    //         l.push_back(i);
    //         p.push_back(curr_cvpoints[i]);
    //         q.push_back(next_cvpoints[i]);
    //     }
    // }

    // 更新下一帧的特征点坐标：将光流跟踪成功的特征点位置保存到下一帧的关键点集合 next_keypoints 中，便于后续使用
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (result_status[i]) {
            next_keypoints[i].x() = next_cvpoints[i].x;
            next_keypoints[i].y() = next_cvpoints[i].y;
        }
    }
}

// 图像预处理，包括直方图均衡化和光流金字塔构建
void OpenCvImage::preprocess(double clipLimit, int width, int height) {
    // 直方图均衡化
    clahe(clipLimit, width, height)->apply(image, image);
    // 构建光流金字塔
    image_pyramid.clear();
    buildOpticalFlowPyramid(image, image_pyramid, Size(21, 21),
                            (int)level_num(), true);
}

void OpenCvImage::correct_distortion(const matrix<3> &intrinsics,
                                     const vector<4> &coeffs) {
    Mat new_image;
    Mat K(3, 3, CV_32FC1), cvcoeffs(1, 4, CV_32FC1);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K.at<float>(i, j) = (float)intrinsics(i, j);
        }
    }
    for (int i = 0; i < 4; ++i) {
        cvcoeffs.at<float>(i) = (float)coeffs(i);
    }
    undistort(image, new_image, K, cvcoeffs);
    image = new_image;
}

CLAHE *OpenCvImage::clahe(double clipLimit, int width, int height) {
    static Ptr<CLAHE> s_clahe = createCLAHE(clipLimit, cv::Size(width, height));
    return s_clahe.get();
}

GFTTDetector *OpenCvImage::gftt(size_t max_points) {
    static Ptr<GFTTDetector> s_gftt =
        GFTTDetector::create(max_points, 1.0e-3, 20, 3, true);
    return s_gftt.get();
}

FastFeatureDetector *OpenCvImage::fast() {
    static Ptr<FastFeatureDetector> s_fast = FastFeatureDetector::create();
    return s_fast.get();
}

ORB *OpenCvImage::orb() {
    static Ptr<ORB> s_orb = ORB::create();
    return s_orb.get();
}

void OpenCvImage::release_image_buffer() {
    image.release();
    raw.release();
    image_pyramid.clear();
    image_levels.clear();
    interpolator_levels.clear();
    grid_levels.clear();
    scale_levels.clear();
}

} // namespace xrslam::extra
