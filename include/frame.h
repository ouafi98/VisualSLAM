#include <Eigen/Dense>


#include <sophus/se3.hpp>
#include <memory>
#include <mutex>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using SE3 = Sophus::SE3d;

struct Frame{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Frame>;

        unsigned long id_ = 0;          // id of this frame
        unsigned long keyframe_id_=0;   // id of keyframe
        bool is_keyframe_ = false;
        double time_stamp_;
        SE3 pose_;                      // pose defined as Tcw
        std::mutex pose_mutex_;
        cv::Mat left_img_, right_img_;  // stereo image


        //feature in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;
    
        Frame(){}
        Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &left, const cv::Mat &right);

        // set and get pose, thread safe
        SE3 GetPose(){
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose){
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        // set keyframe and keyframe_id
        void SetKeyFrame();

        // create a new frame and allocate id
        static std::shared_ptr<Frame> CreateFrame()

};