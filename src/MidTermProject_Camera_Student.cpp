/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;


vector<Config2DFeatTrack> getConfig(bool singleTest) {

    vector<Config2DFeatTrack> configList;
    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    vector<string> matcherTypes = {"MAT_BF", "MAT_FLANN"};
    vector<string> matcherTypeMetrics = {"DES_BINARY", "DES_HOG"};
    vector<string> matcherTypeSelectors = {"SEL_NN", "SEL_KNN"};

    if (singleTest) {
        Config2DFeatTrack config;
        config.detectorType = detectorTypes[0];
        config.descriptorType = descriptorTypes[0];
        config.matcherType = matcherTypes[0];
        config.matcherTypeMetric = matcherTypeMetrics[0];
        config.matcherTypeSelector = matcherTypeSelectors[0];

        config.bVis = true;
        config.bLimitKpts = true;
        config.maxKeypoints = 50;

        configList.push_back(config);
    } else {
        for (auto detectorType:detectorTypes) {
            bool write_detector = false;

            for (auto descriptorType:descriptorTypes) // start
            {
                if (descriptorType.compare("AKAZE") == 0)
                    continue;

                for (auto matcherType:matcherTypes) {
                    for (auto matcherTypeMetric:matcherTypeMetrics) {
                        for (auto matcherTypeSelector:matcherTypeSelectors) {
                            Config2DFeatTrack config;
                            config.detectorType = detectorType;
                            config.descriptorType = descriptorType;
                            config.matcherType = matcherType;
                            config.matcherTypeMetric = matcherTypeMetric;
                            config.matcherTypeSelector = matcherTypeSelector;

                            configList.push_back(config);
                        }
                    }
                }
            }
        }

    }
    return configList;
}


void log(AuditLog &audit) {

    cout << "{" << endl;
    cout << "detectorType:" << audit.config.detectorType << endl;
    cout << "descriptorType:" << audit.config.descriptorType << endl;
    cout << "matcherType:" << audit.config.matcherType << endl;
    cout << "matcherTypeMetric:" << audit.config.matcherTypeMetric << endl;
    cout << "matcherTypeSelector:" << audit.config.matcherTypeSelector << endl;
    cout << "bVis:" << audit.config.bVis << endl;
    cout << "bLimitKpts:" << audit.config.bLimitKpts << endl;
    cout << "maxKeypoints:" << audit.config.maxKeypoints << endl;

    cout << "match_time:" << audit.match_time << endl;
    cout << "match_keypoints_size:" << audit.match_keypoints_size << endl;
    cout << "match_removed_keypoints_size:" << audit.match_removed_keypoints_size << endl;

    cout << "desc_time:" << audit.desc_time << endl;

    cout << "detect_time:" << audit.detect_time << endl;
    cout << "detect_keypoints_size:" << audit.detect_keypoints_size << endl;

    cout << "}" << endl;
}

void log_audit_header(ofstream &detector_file) {
    detector_file << "error";
    detector_file << "detectorType";
    detector_file << "," << "descriptorType";
    detector_file << "," << "matcherType";
    detector_file << "," << "matcherTypeMetric";
    detector_file << "," << "matcherTypeSelector";
    detector_file << "," << "bVis";
    detector_file << "," << "bLimitKpts";
    detector_file << "," << "maxKeypoints";

    detector_file << "," << "match_time";
    detector_file << "," << "match_keypoints_size";
    detector_file << "," << "match_removed_keypoints_size";

    detector_file << "," << "desc_time";

    detector_file << "," << "detect_time";
    detector_file << "," << "detect_keypoints_size" << endl;
}

void log_audit(ofstream &detector_file, AuditLog &audit) {

    detector_file << audit.isError;
    detector_file << audit.config.detectorType;
    detector_file << "," << audit.config.descriptorType;
    detector_file << "," << audit.config.matcherType;
    detector_file << "," << audit.config.matcherTypeMetric;
    detector_file << "," << audit.config.matcherTypeSelector;
    detector_file << "," << audit.config.bVis;
    detector_file << "," << audit.config.bLimitKpts;
    detector_file << "," << audit.config.maxKeypoints;

    detector_file << "," << audit.match_time;
    detector_file << "," << audit.match_keypoints_size;
    detector_file << "," << audit.match_removed_keypoints_size;

    detector_file << "," << audit.desc_time;

    detector_file << "," << audit.detect_time;
    detector_file << "," << audit.detect_keypoints_size << endl;
}

//void log_audits(vector<AuditLog> &audits){
//
//    for(auto audit=audits.begin(); audit != audits.end(); ++audit)
//    {
//        log_audit(detector_file, (*audit));
//    }
//}

/* MAIN PROGRAM */
int run_2D_tracking(Config2DFeatTrack &config2d, AuditLog &audit) {

    /* INIT VARIABLES AND DATA STRUCTURES */


    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //audit
        audit.image_name = imgNumber.str();
        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        if (dataBuffer.size() > dataBufferSize) {
            cout << "dataBuffer size before removal is =" << dataBuffer.size() << endl;
            dataBuffer.erase(dataBuffer.begin());
        }
        cout << "#1 : LOAD IMAGE INTO BUFFER done. dataBuffer size is =" << dataBuffer.size() << endl;

        //// EOF STUDENT ASSIGNMENT


        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = config2d.detectorType;//"SHITOMASI";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        std::vector<string> detectors{"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
        if (std::find(detectors.begin(), detectors.end(), detectorType) != detectors.end()) {
            cout << "detectorType " << detectorType << endl;
        } else {
            cout << "Invalid detectorType " << detectorType << endl;
            return -1;
        }

        if (detectorType.compare("SHITOMASI") == 0) {
            detKeypointsShiTomasi(keypoints, imgGray, config2d, audit, false);
        } else if (detectorType.compare("HARRIS") == 0) {
            detKeypointsHarris(keypoints, imgGray, config2d, audit, false);
        } else {
            // FAST, BRISK, ORB, AKAZE, SIFT
            detKeypointsModern(keypoints, imgGray, detectorType, config2d, audit, false);
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle) {
            vector<cv::KeyPoint> keypointsRoi;
            for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
                if (vehicleRect.contains(it->pt)) {
                    keypointsRoi.push_back((*it));
                }
            }
            keypoints = keypointsRoi;
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = config2d.bLimitKpts;//false;
        if (bLimitKpts) {
            int maxKeypoints = config2d.maxKeypoints;// 50;

            if (detectorType.compare("SHITOMASI") ==
                0) { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType,
                      config2d, audit);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;

            string matcherType = config2d.matcherType;//"MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = config2d.matcherTypeMetric;//"DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = config2d.matcherTypeSelector;//"SEL_NN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType, config2d, audit);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = config2d.bVis; //true;
            if (bVis) {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    return 0;
}


int main(int argc, const char *argv[]) {
    ofstream detector_file;
    detector_file.open("../results.csv");
    bool singleTest = false;
    //int run_2D_tracking(Config2DFeatTrack &config, vector<AuditLog> &audits)
    vector<Config2DFeatTrack> configList = getConfig(singleTest);
    vector<AuditLog> audits;
    log_audit_header(detector_file);

    for (auto config2d = configList.begin(); config2d != configList.end(); ++config2d) {
        AuditLog audit;
        audit.config = (*config2d);
        audits.push_back(audit);
        try {
            run_2D_tracking((*config2d), audit);
            log(audit);
            log_audit(detector_file, audit);
        } catch (...) {
            cout << "exception happened" << endl;
            audit.isError = true;
            log(audit);
            log_audit(detector_file, audit);
        }
    }
//    log_audits(audits);
    detector_file.close();
}