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

 ## anylysis
 
 [report_analysis.md](./report_analysis.md)
  
 [report_analysis.html](./report_analysis.html)
 
 
 
```python
import pandas as pd
import numpy as np
import os.path
```


```python
def get_image_name(path): 
    return os.path.basename(path)
```


```python
df = pd.read_csv("results.csv")
df['image_name'] = df['image_name'].apply(get_image_name)
del df['error']
df.head(3)
```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th>image_name</th>
      <th>detectorType</th>
      <th>descriptorType</th>
      <th>matcherType</th>
      <th>matcherTypeMetric</th>
      <th>matcherTypeSelector</th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>0</th>
      <td>0000000000.png</td>
      <td>SHITOMASI</td>
      <td>BRISK</td>
      <td>MAT_BF</td>
      <td>DES_BINARY</td>
      <td>SEL_NN</td>
      <td>19</td>
      <td>339</td>
      <td>0</td>
      <td>1370</td>
      <td>0</td>
      <td>0</td>
      <td>0</td>
      <td>0</td>
      <td>50</td>
    </tr>
    <tr>
      <th>1</th>
      <td>0000000001.png</td>
      <td>SHITOMASI</td>
      <td>BRISK</td>
      <td>MAT_BF</td>
      <td>DES_BINARY</td>
      <td>SEL_NN</td>
      <td>17</td>
      <td>343</td>
      <td>0</td>
      <td>1301</td>
      <td>125</td>
      <td>0</td>
      <td>0</td>
      <td>0</td>
      <td>50</td>
    </tr>
    <tr>
      <th>2</th>
      <td>0000000002.png</td>
      <td>SHITOMASI</td>
      <td>BRISK</td>
      <td>MAT_BF</td>
      <td>DES_BINARY</td>
      <td>SEL_NN</td>
      <td>17</td>
      <td>337</td>
      <td>0</td>
      <td>1361</td>
      <td>118</td>
      <td>0</td>
      <td>0</td>
      <td>0</td>
      <td>50</td>
    </tr>
  </tbody>
</table>
</div>



detect_time by detectorType


```python
df_detect= df.groupby(['detectorType']).mean()
df_detect.sort_values(by=['detect_time'], ascending=[1])
 
```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>detectorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>FAST</th>
      <td>0.933333</td>
      <td>65.425000</td>
      <td>0.739583</td>
      <td>1787.4</td>
      <td>84.781250</td>
      <td>16.318750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>7.445833</td>
      <td>69.620833</td>
      <td>0.427083</td>
      <td>500.0</td>
      <td>55.825000</td>
      <td>12.200000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>14.931250</td>
      <td>65.900000</td>
      <td>0.543750</td>
      <td>1342.3</td>
      <td>69.845833</td>
      <td>10.179167</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>HARRIS</th>
      <td>16.062500</td>
      <td>64.222917</td>
      <td>0.000000</td>
      <td>173.7</td>
      <td>13.500000</td>
      <td>2.550000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>72.608333</td>
      <td>75.564583</td>
      <td>0.972917</td>
      <td>1342.9</td>
      <td>121.081250</td>
      <td>15.593750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <td>124.356250</td>
      <td>71.014583</td>
      <td>0.531250</td>
      <td>1386.2</td>
      <td>54.454167</td>
      <td>18.220833</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>365.677083</td>
      <td>68.922917</td>
      <td>1.629167</td>
      <td>2711.6</td>
      <td>145.668750</td>
      <td>39.397917</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



detect_time by descriptorType 


```python
df_detect2= df.groupby(['descriptorType']).mean()
df_detect2.sort_values(by=['detect_time'], ascending=[1])
```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>descriptorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>SIFT</th>
      <td>82.450000</td>
      <td>35.317857</td>
      <td>1.037500</td>
      <td>1320.585714</td>
      <td>55.564286</td>
      <td>8.078571</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <td>85.591071</td>
      <td>40.201786</td>
      <td>0.776786</td>
      <td>1320.585714</td>
      <td>93.953571</td>
      <td>23.675000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>86.833929</td>
      <td>325.205357</td>
      <td>0.864286</td>
      <td>1320.585714</td>
      <td>102.894643</td>
      <td>23.191071</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>86.875000</td>
      <td>8.635714</td>
      <td>0.128571</td>
      <td>1320.585714</td>
      <td>19.337500</td>
      <td>1.962500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <td>86.998214</td>
      <td>0.608929</td>
      <td>0.667857</td>
      <td>1320.585714</td>
      <td>105.757143</td>
      <td>21.528571</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>87.264286</td>
      <td>2.033929</td>
      <td>0.676786</td>
      <td>1320.585714</td>
      <td>89.769643</td>
      <td>19.673214</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



desc_time by detectorType


```python
df_detect= df.groupby(['detectorType']).mean()
df_detect.sort_values(by=['desc_time'], ascending=[1])

```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>detectorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>HARRIS</th>
      <td>16.062500</td>
      <td>64.222917</td>
      <td>0.000000</td>
      <td>173.7</td>
      <td>13.500000</td>
      <td>2.550000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FAST</th>
      <td>0.933333</td>
      <td>65.425000</td>
      <td>0.739583</td>
      <td>1787.4</td>
      <td>84.781250</td>
      <td>16.318750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>14.931250</td>
      <td>65.900000</td>
      <td>0.543750</td>
      <td>1342.3</td>
      <td>69.845833</td>
      <td>10.179167</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>365.677083</td>
      <td>68.922917</td>
      <td>1.629167</td>
      <td>2711.6</td>
      <td>145.668750</td>
      <td>39.397917</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>7.445833</td>
      <td>69.620833</td>
      <td>0.427083</td>
      <td>500.0</td>
      <td>55.825000</td>
      <td>12.200000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <td>124.356250</td>
      <td>71.014583</td>
      <td>0.531250</td>
      <td>1386.2</td>
      <td>54.454167</td>
      <td>18.220833</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>72.608333</td>
      <td>75.564583</td>
      <td>0.972917</td>
      <td>1342.9</td>
      <td>121.081250</td>
      <td>15.593750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



desc_time by descriptorType


```python
df_detect= df.groupby(['descriptorType']).mean()
df_detect.sort_values(by=['desc_time'], ascending=[1])

```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>descriptorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>BRIEF</th>
      <td>86.998214</td>
      <td>0.608929</td>
      <td>0.667857</td>
      <td>1320.585714</td>
      <td>105.757143</td>
      <td>21.528571</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>87.264286</td>
      <td>2.033929</td>
      <td>0.676786</td>
      <td>1320.585714</td>
      <td>89.769643</td>
      <td>19.673214</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>86.875000</td>
      <td>8.635714</td>
      <td>0.128571</td>
      <td>1320.585714</td>
      <td>19.337500</td>
      <td>1.962500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <td>82.450000</td>
      <td>35.317857</td>
      <td>1.037500</td>
      <td>1320.585714</td>
      <td>55.564286</td>
      <td>8.078571</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <td>85.591071</td>
      <td>40.201786</td>
      <td>0.776786</td>
      <td>1320.585714</td>
      <td>93.953571</td>
      <td>23.675000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>86.833929</td>
      <td>325.205357</td>
      <td>0.864286</td>
      <td>1320.585714</td>
      <td>102.894643</td>
      <td>23.191071</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



detect_time by descriptorType, detectorType


```python
df_detect3= df.groupby(['descriptorType', 'detectorType']).mean()
df_detect3.sort_values(by=['detect_time'], ascending=[0])

```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>descriptorType</th>
      <th>detectorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>ORB</th>
      <th>BRISK</th>
      <td>369.7000</td>
      <td>4.0750</td>
      <td>1.8500</td>
      <td>2711.6</td>
      <td>187.4500</td>
      <td>63.3500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>BRISK</th>
      <td>367.4750</td>
      <td>1.0125</td>
      <td>1.6750</td>
      <td>2711.6</td>
      <td>202.2250</td>
      <td>48.5750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>BRISK</th>
      <td>366.1625</td>
      <td>327.3625</td>
      <td>2.1500</td>
      <td>2711.6</td>
      <td>197.6250</td>
      <td>53.1750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>BRISK</th>
      <td>364.1625</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>2711.6</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>BRISK</th>
      <td>363.5875</td>
      <td>40.2000</td>
      <td>1.8875</td>
      <td>2711.6</td>
      <td>182.4250</td>
      <td>50.1750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>BRISK</th>
      <td>362.9750</td>
      <td>40.8875</td>
      <td>2.2125</td>
      <td>2711.6</td>
      <td>104.2875</td>
      <td>21.1125</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>SIFT</th>
      <td>128.9125</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1386.2</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>SIFT</th>
      <td>128.7250</td>
      <td>39.4625</td>
      <td>0.7875</td>
      <td>1386.2</td>
      <td>89.5625</td>
      <td>34.3375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>SIFT</th>
      <td>127.7625</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1386.2</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>SIFT</th>
      <td>127.7500</td>
      <td>0.5500</td>
      <td>0.5750</td>
      <td>1386.2</td>
      <td>95.1125</td>
      <td>29.7875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>SIFT</th>
      <td>127.0875</td>
      <td>305.2000</td>
      <td>0.8750</td>
      <td>1386.2</td>
      <td>90.7375</td>
      <td>34.0625</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>SIFT</th>
      <td>105.9000</td>
      <td>80.8750</td>
      <td>0.9500</td>
      <td>1386.2</td>
      <td>51.3125</td>
      <td>11.1375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>AKAZE</th>
      <td>73.9875</td>
      <td>325.3750</td>
      <td>0.9500</td>
      <td>1342.9</td>
      <td>132.8250</td>
      <td>16.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>AKAZE</th>
      <td>73.3750</td>
      <td>60.4500</td>
      <td>0.9000</td>
      <td>1342.9</td>
      <td>135.3625</td>
      <td>13.7375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>AKAZE</th>
      <td>73.0750</td>
      <td>3.1875</td>
      <td>0.9625</td>
      <td>1342.9</td>
      <td>127.0250</td>
      <td>22.0750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>AKAZE</th>
      <td>72.7750</td>
      <td>0.9500</td>
      <td>0.8000</td>
      <td>1342.9</td>
      <td>133.4750</td>
      <td>15.6250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>AKAZE</th>
      <td>72.2250</td>
      <td>39.6500</td>
      <td>0.9250</td>
      <td>1342.9</td>
      <td>128.5750</td>
      <td>20.5250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>AKAZE</th>
      <td>70.2125</td>
      <td>23.7750</td>
      <td>1.3000</td>
      <td>1342.9</td>
      <td>69.2250</td>
      <td>5.3250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>HARRIS</th>
      <td>17.8000</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>SHITOMASI</th>
      <td>16.9000</td>
      <td>336.1125</td>
      <td>0.6625</td>
      <td>1342.3</td>
      <td>89.7375</td>
      <td>16.9625</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>HARRIS</th>
      <td>16.7500</td>
      <td>15.7500</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>9.4250</td>
      <td>1.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="2" valign="top">BRIEF</th>
      <th>HARRIS</th>
      <td>16.7000</td>
      <td>0.0750</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>18.5500</td>
      <td>2.8500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>16.2125</td>
      <td>1.0125</td>
      <td>0.5000</td>
      <td>1342.3</td>
      <td>97.3000</td>
      <td>9.4000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>SHITOMASI</th>
      <td>16.1375</td>
      <td>1.0000</td>
      <td>0.5750</td>
      <td>1342.3</td>
      <td>95.2750</td>
      <td>11.4250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>SHITOMASI</th>
      <td>15.8750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1342.3</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>HARRIS</th>
      <td>15.4750</td>
      <td>327.7625</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>17.2750</td>
      <td>4.1250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>HARRIS</th>
      <td>15.0125</td>
      <td>0.6500</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>18.3750</td>
      <td>3.0250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>HARRIS</th>
      <td>14.6375</td>
      <td>41.1000</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>17.3750</td>
      <td>4.0250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>SHITOMASI</th>
      <td>12.7250</td>
      <td>16.0125</td>
      <td>0.9125</td>
      <td>1342.3</td>
      <td>49.8500</td>
      <td>3.5000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>SHITOMASI</th>
      <td>11.7375</td>
      <td>41.2625</td>
      <td>0.6125</td>
      <td>1342.3</td>
      <td>86.9125</td>
      <td>19.7875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>ORB</th>
      <td>8.1750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>500.0</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>ORB</th>
      <td>7.5875</td>
      <td>47.6750</td>
      <td>0.7375</td>
      <td>500.0</td>
      <td>44.9750</td>
      <td>6.6750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>ORB</th>
      <td>7.2625</td>
      <td>39.0625</td>
      <td>0.3250</td>
      <td>500.0</td>
      <td>46.6250</td>
      <td>8.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>ORB</th>
      <td>7.2375</td>
      <td>0.1875</td>
      <td>0.4500</td>
      <td>500.0</td>
      <td>76.5625</td>
      <td>26.7375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>ORB</th>
      <td>7.2250</td>
      <td>326.4750</td>
      <td>0.5125</td>
      <td>500.0</td>
      <td>82.5750</td>
      <td>12.4250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>ORB</th>
      <td>7.1875</td>
      <td>4.3250</td>
      <td>0.5375</td>
      <td>500.0</td>
      <td>84.2125</td>
      <td>19.0875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <th>FAST</th>
      <td>1.0000</td>
      <td>328.1500</td>
      <td>0.9000</td>
      <td>1787.4</td>
      <td>109.4875</td>
      <td>25.3125</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>FAST</th>
      <td>1.0000</td>
      <td>22.2500</td>
      <td>1.1500</td>
      <td>1787.4</td>
      <td>59.8750</td>
      <td>7.5250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>FAST</th>
      <td>0.9750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1787.4</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FREAK</th>
      <th>FAST</th>
      <td>0.9625</td>
      <td>40.6750</td>
      <td>0.9000</td>
      <td>1787.4</td>
      <td>106.2000</td>
      <td>28.6000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>FAST</th>
      <td>0.8375</td>
      <td>0.4750</td>
      <td>0.6750</td>
      <td>1787.4</td>
      <td>117.0750</td>
      <td>17.7250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>FAST</th>
      <td>0.8250</td>
      <td>1.0000</td>
      <td>0.8125</td>
      <td>1787.4</td>
      <td>116.0500</td>
      <td>18.7500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



desc_time by descriptorType, detectorType


```python
df_detect3= df.groupby(['descriptorType', 'detectorType']).mean()
df_detect3.sort_values(by=['desc_time'], ascending=[0]) 
```




<div>
<table border="1" class="dataframe">
  <thead>
    <tr style="text-align: right;">
      <th></th>
      <th></th>
      <th>detect_time</th>
      <th>desc_time</th>
      <th>match_time</th>
      <th>detect_keypoints_size</th>
      <th>match_keypoints_size</th>
      <th>match_removed_keypoints_size</th>
      <th>bVis</th>
      <th>bLimitKpts</th>
      <th>maxKeypoints</th>
    </tr>
    <tr>
      <th>descriptorType</th>
      <th>detectorType</th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
      <th></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th rowspan="7" valign="top">BRISK</th>
      <th>SHITOMASI</th>
      <td>16.9000</td>
      <td>336.1125</td>
      <td>0.6625</td>
      <td>1342.3</td>
      <td>89.7375</td>
      <td>16.9625</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FAST</th>
      <td>1.0000</td>
      <td>328.1500</td>
      <td>0.9000</td>
      <td>1787.4</td>
      <td>109.4875</td>
      <td>25.3125</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>HARRIS</th>
      <td>15.4750</td>
      <td>327.7625</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>17.2750</td>
      <td>4.1250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>366.1625</td>
      <td>327.3625</td>
      <td>2.1500</td>
      <td>2711.6</td>
      <td>197.6250</td>
      <td>53.1750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>7.2250</td>
      <td>326.4750</td>
      <td>0.5125</td>
      <td>500.0</td>
      <td>82.5750</td>
      <td>12.4250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>73.9875</td>
      <td>325.3750</td>
      <td>0.9500</td>
      <td>1342.9</td>
      <td>132.8250</td>
      <td>16.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <td>127.0875</td>
      <td>305.2000</td>
      <td>0.8750</td>
      <td>1386.2</td>
      <td>90.7375</td>
      <td>34.0625</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>SIFT</th>
      <td>105.9000</td>
      <td>80.8750</td>
      <td>0.9500</td>
      <td>1386.2</td>
      <td>51.3125</td>
      <td>11.1375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>AKAZE</th>
      <td>73.3750</td>
      <td>60.4500</td>
      <td>0.9000</td>
      <td>1342.9</td>
      <td>135.3625</td>
      <td>13.7375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>ORB</th>
      <td>7.5875</td>
      <td>47.6750</td>
      <td>0.7375</td>
      <td>500.0</td>
      <td>44.9750</td>
      <td>6.6750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="2" valign="top">FREAK</th>
      <th>SHITOMASI</th>
      <td>11.7375</td>
      <td>41.2625</td>
      <td>0.6125</td>
      <td>1342.3</td>
      <td>86.9125</td>
      <td>19.7875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>HARRIS</th>
      <td>14.6375</td>
      <td>41.1000</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>17.3750</td>
      <td>4.0250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <th>BRISK</th>
      <td>362.9750</td>
      <td>40.8875</td>
      <td>2.2125</td>
      <td>2711.6</td>
      <td>104.2875</td>
      <td>21.1125</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="5" valign="top">FREAK</th>
      <th>FAST</th>
      <td>0.9625</td>
      <td>40.6750</td>
      <td>0.9000</td>
      <td>1787.4</td>
      <td>106.2000</td>
      <td>28.6000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>363.5875</td>
      <td>40.2000</td>
      <td>1.8875</td>
      <td>2711.6</td>
      <td>182.4250</td>
      <td>50.1750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>72.2250</td>
      <td>39.6500</td>
      <td>0.9250</td>
      <td>1342.9</td>
      <td>128.5750</td>
      <td>20.5250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SIFT</th>
      <td>128.7250</td>
      <td>39.4625</td>
      <td>0.7875</td>
      <td>1386.2</td>
      <td>89.5625</td>
      <td>34.3375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>7.2625</td>
      <td>39.0625</td>
      <td>0.3250</td>
      <td>500.0</td>
      <td>46.6250</td>
      <td>8.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="4" valign="top">SIFT</th>
      <th>AKAZE</th>
      <td>70.2125</td>
      <td>23.7750</td>
      <td>1.3000</td>
      <td>1342.9</td>
      <td>69.2250</td>
      <td>5.3250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FAST</th>
      <td>1.0000</td>
      <td>22.2500</td>
      <td>1.1500</td>
      <td>1787.4</td>
      <td>59.8750</td>
      <td>7.5250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>12.7250</td>
      <td>16.0125</td>
      <td>0.9125</td>
      <td>1342.3</td>
      <td>49.8500</td>
      <td>3.5000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>HARRIS</th>
      <td>16.7500</td>
      <td>15.7500</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>9.4250</td>
      <td>1.2750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="3" valign="top">ORB</th>
      <th>ORB</th>
      <td>7.1875</td>
      <td>4.3250</td>
      <td>0.5375</td>
      <td>500.0</td>
      <td>84.2125</td>
      <td>19.0875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>369.7000</td>
      <td>4.0750</td>
      <td>1.8500</td>
      <td>2711.6</td>
      <td>187.4500</td>
      <td>63.3500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <td>73.0750</td>
      <td>3.1875</td>
      <td>0.9625</td>
      <td>1342.9</td>
      <td>127.0250</td>
      <td>22.0750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="2" valign="top">BRIEF</th>
      <th>SHITOMASI</th>
      <td>16.2125</td>
      <td>1.0125</td>
      <td>0.5000</td>
      <td>1342.3</td>
      <td>97.3000</td>
      <td>9.4000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>367.4750</td>
      <td>1.0125</td>
      <td>1.6750</td>
      <td>2711.6</td>
      <td>202.2250</td>
      <td>48.5750</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="2" valign="top">ORB</th>
      <th>FAST</th>
      <td>0.8250</td>
      <td>1.0000</td>
      <td>0.8125</td>
      <td>1787.4</td>
      <td>116.0500</td>
      <td>18.7500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>16.1375</td>
      <td>1.0000</td>
      <td>0.5750</td>
      <td>1342.3</td>
      <td>95.2750</td>
      <td>11.4250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRIEF</th>
      <th>AKAZE</th>
      <td>72.7750</td>
      <td>0.9500</td>
      <td>0.8000</td>
      <td>1342.9</td>
      <td>133.4750</td>
      <td>15.6250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>HARRIS</th>
      <td>15.0125</td>
      <td>0.6500</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>18.3750</td>
      <td>3.0250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="4" valign="top">BRIEF</th>
      <th>SIFT</th>
      <td>127.7500</td>
      <td>0.5500</td>
      <td>0.5750</td>
      <td>1386.2</td>
      <td>95.1125</td>
      <td>29.7875</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FAST</th>
      <td>0.8375</td>
      <td>0.4750</td>
      <td>0.6750</td>
      <td>1787.4</td>
      <td>117.0750</td>
      <td>17.7250</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>7.2375</td>
      <td>0.1875</td>
      <td>0.4500</td>
      <td>500.0</td>
      <td>76.5625</td>
      <td>26.7375</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>HARRIS</th>
      <td>16.7000</td>
      <td>0.0750</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>18.5500</td>
      <td>2.8500</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>AKAZE</th>
      <th>HARRIS</th>
      <td>17.8000</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>173.7</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <th>SIFT</th>
      <td>128.9125</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1386.2</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th rowspan="5" valign="top">AKAZE</th>
      <th>SIFT</th>
      <td>127.7625</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1386.2</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>SHITOMASI</th>
      <td>15.8750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1342.3</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>BRISK</th>
      <td>364.1625</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>2711.6</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>FAST</th>
      <td>0.9750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>1787.4</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
    <tr>
      <th>ORB</th>
      <td>8.1750</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>500.0</td>
      <td>0.0000</td>
      <td>0.0000</td>
      <td>0.0</td>
      <td>0.0</td>
      <td>50.0</td>
    </tr>
  </tbody>
</table>
</div>



 