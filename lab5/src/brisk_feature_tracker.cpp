// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2024  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a BRISK feature tracker derived class
//  that inherits from your FeatureTracker base class.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// For this part, you will need to implement the same functions you've just
// implemented in the case of SIFT, AKAZE, and ORB, but now for BRISK.
//
// NOTE: We do not provide much template code for this section or the
// corresponding header file (brisk_feature_tracker.h), but you should define
// and implement the same set of functions as for SIFT and AKAZE (see the
// template code for those cases for reference as needed)
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "brisk_feature_tracker.h"

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

using namespace cv;

BriskFeatureTracker::BriskFeatureTracker()
    : FeatureTracker(), detector(BRISK::create()) {}

void BriskFeatureTracker::detectKeypoints(const cv::Mat& img,
                                          std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  BriskFeatureTracker::detector->detect(img , *keypoints); 
  //
  // ~~~~ end solution
}

void BriskFeatureTracker::describeKeypoints(const cv::Mat& img,
                                            std::vector<KeyPoint>* keypoints,
                                            cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  BriskFeatureTracker::detector->detectAndCompute(img , noArray() , *keypoints, *descriptors );
  //
  // ~~~~ end solution
}

void BriskFeatureTracker::matchDescriptors(
    const cv::Mat& descriptors_1,
    const cv::Mat& descriptors_2,
    std::vector<std::vector<DMatch>>* matches,
    std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);

  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  // FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  matcher->knnMatch( descriptors_1, descriptors_2, *matches , 2 );

  const float ratio = 0.8f; 
  for (size_t i =0 ; i< matches->size(); i++){
    if ( matches->at(i)[0].distance < ratio * matches->at(i)[1].distance)
        {
            good_matches->push_back(matches->at(i)[0]);
        }
  }
  //
  // ~~~~ end solution
}
