#include <Eigen/Dense>



#include <memory>
#include <mutex>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "frame.h"


struct Feature{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Feature>;

    std::weak_ptr<Frame> frame_; // frame that takes this feature
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_; // assigned map point

    bool is_oulier = false;
    bool is_on_left_image_ = true;

    Feature(){}
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp): frame_(frame), position(kp) {}


};