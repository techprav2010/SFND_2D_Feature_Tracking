#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct Config2DFeatTrack {
    std::string detectorType = "SHITOMASI";
    std::string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT

    std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    std::string matcherTypeMetric = "DES_BINARY"; // DES_BINARY, DES_HOG
    std::string matcherTypeSelector = "SEL_NN";       // SEL_NN, SEL_KNN

    bool bVis = false;
    bool bLimitKpts = false;
    int maxKeypoints = 50;
};

struct AuditLog {
    Config2DFeatTrack config ;
    std::string image_name;
    bool isError=false;
    long match_time, match_keypoints_size, match_removed_keypoints_size;
    long desc_time;
    long detect_time, detect_keypoints_size, detect_removed_keypoints_size;
};
#endif /* dataStructures_h */
