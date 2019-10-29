# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.




## Code
* I added code for variuos opencv algorithms for - keypoint detectors, descriptor, and matchers from two consecutive images.
* I used  KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8 in file `matching2D.cpp`.
* Created test results by running combination of algorithms for - keypoint detectors, descriptor, and matchers against KITTI images.

## Benchmark

### audit log for m looping multiple options and collect experiment data
* Added new code to capture KPI, metrics from various experiments (with combinations of alogrithms detectors, descriptors and match algorithms).
* Used 'struct' to keep track of experiments - Config2DFeatTrack and AuditLog 
    ```c++  
        
        //struct to hold experiment configuration
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
    
        //struct to hold experiment audit log
        struct AuditLog {
            Config2DFeatTrack config ;
            std::string image_name ="";
            bool isError = false;
            long match_time = 0;
            long match_keypoints_size = 0;
            long match_removed_keypoints_size = 0;
            long desc_time  = 0;
            long detect_time = 0;
            long detect_keypoints_size = 0;
        };
   ```
* Now dyanamically create a new array of Config2DFeatTrack  with all possible combinations from following setting.
    
    ```c++
  
        vector<Config2DFeatTrack> configList;
  
        vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
        vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"}; 
        vector<string> matcherTypes = {"MAT_BF", "MAT_FLANN"};
        vector<string> matcherTypeMetrics = {"DES_BINARY", "DES_HOG"};
        vector<string> matcherTypeSelectors = {"SEL_NN", "SEL_KNN"};
  
        ...
        ...
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
        ...
        ...

     ```
 * To run all combinations of algorithms or one set of algorithm. 
    Please change 'singleTest' in 'MidTermProject_Camera_Student.cpp -> main'
    ```c++
       //'MidTermProject_Camera_Student.cpp -> main'
        // for testing one set of algorithms use singleTest = true
       //if singleTest = false,  will run all combinations of algorithms
       bool singleTest = false;
 
       vector<Config2DFeatTrack> configList = getConfig(singleTest);
   
       ```
   
 * output of log outputed into three files
    runtest.log
    
    results.csv
    ```
        error,image_name,detectorType,descriptorType,matcherType,matcherTypeMetric,matcherTypeSelector,detect_time,desc_time,match_time,detect_keypoints_size,match_keypoints_size,match_removed_keypoints_size,bVis,bLimitKpts,maxKeypoints
        0,../images/KITTI/2011_09_26/image_00/data/0000000000.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,19,345,0,1370,0,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000001.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,19,346,0,1301,125,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000002.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,19,339,0,1361,118,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000003.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,16,338,0,1358,123,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000004.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,16,336,0,1333,120,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000005.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,17,338,0,1284,120,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000006.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,17,336,0,1322,113,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000007.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,16,337,0,1366,114,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000008.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,17,336,0,1389,123,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000009.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_NN,17,337,0,1339,111,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000000.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_KNN,16,336,0,1370,0,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000001.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_KNN,17,334,0,1301,95,30,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000002.png,SHITOMASI,BRISK,MAT_BF,DES_BINARY,SEL_KNN,17,334,0,1361,88,30,0,0,50
         
        0,../images/KITTI/2011_09_26/image_00/data/0000000007.png,SHITOMASI,SIFT,MAT_FLANN,DES_BINARY,SEL_KNN,11,17,3,1366,96,18,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000008.png,SHITOMASI,SIFT,MAT_FLANN,DES_BINARY,SEL_KNN,11,19,2,1389,106,17,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000009.png,SHITOMASI,SIFT,MAT_FLANN,DES_BINARY,SEL_KNN,17,15,2,1339,97,14,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000000.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,11,19,0,1370,0,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000001.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,11,16,2,1301,125,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000002.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,11,15,2,1361,118,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000003.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,16,15,2,1358,123,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000004.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,11,15,2,1333,120,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000005.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,12,16,2,1284,120,0,0,0,50  
        0,../images/KITTI/2011_09_26/image_00/data/0000000006.png,SHITOMASI,SIFT,MAT_FLANN,DES_HOG,SEL_NN,11,15,1,1322,113,0,0,0,50  

        0,../images/KITTI/2011_09_26/image_00/data/0000000000.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,327,0,1824,0,0,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000001.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,328,0,1832,97,52,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000002.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,329,0,1810,104,48,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000003.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,334,0,1817,101,49,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000004.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,328,0,1793,98,57,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000005.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,332,0,1796,85,64,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000006.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,329,0,1788,107,42,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000007.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,327,0,1695,107,49,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000008.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,325,0,1749,100,50,0,0,50
        0,../images/KITTI/2011_09_26/image_00/data/0000000009.png,FAST,BRISK,MAT_BF,DES_BINARY,SEL_KNN,1,326,0,1770,100,38,0,0,50
              
 
      
   
    ```
     results.json
    ```
           [
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000000.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 19,
               'desc_time_ms': 339,
               'match_time_ms': 0,
               'detect_keypoints_size': 1370,
               'match_keypoints_size': 0,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000001.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 17,
               'desc_time_ms': 343,
               'match_time_ms': 0,
               'detect_keypoints_size': 1301,
               'match_keypoints_size': 125,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000002.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 17,
               'desc_time_ms': 337,
               'match_time_ms': 0,
               'detect_keypoints_size': 1361,
               'match_keypoints_size': 118,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000003.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 17,
               'desc_time_ms': 341,
               'match_time_ms': 0,
               'detect_keypoints_size': 1358,
               'match_keypoints_size': 123,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000004.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 16,
               'desc_time_ms': 336,
               'match_time_ms': 0,
               'detect_keypoints_size': 1333,
               'match_keypoints_size': 120,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000005.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 18,
               'desc_time_ms': 341,
               'match_time_ms': 0,
               'detect_keypoints_size': 1284,
               'match_keypoints_size': 120,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000006.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 16,
               'desc_time_ms': 342,
               'match_time_ms': 0,
               'detect_keypoints_size': 1322,
               'match_keypoints_size': 113,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000007.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 16,
               'desc_time_ms': 339,
               'match_time_ms': 0,
               'detect_keypoints_size': 1366,
               'match_keypoints_size': 114,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
             {
               'isError': '0',
               'image_name': '../images/KITTI/2011_09_26/image_00/data/0000000008.png',
               'detectorType': 'SHITOMASI',
               'descriptorType': 'BRISK',
               'matcherType': 'MAT_BF',
               'matcherTypeMetric': 'DES_BINARY',
               'matcherTypeSelector': 'SEL_NN',
               'detect_time_ms': 17,
               'desc_time_ms': 339,
               'match_time_ms': 0,
               'detect_keypoints_size': 1389,
               'match_keypoints_size': 123,
               'match_removed_keypoints_size': 0,
               'bVis': 0,
               'bLimitKpts': 0,
               'maxKeypoints': 50,
               
             },
            ]
    ``` 
 
 ## results
     
[results.csv](./results.csv)

[results.json](./results.json)
