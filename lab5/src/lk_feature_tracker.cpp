#include "lk_feature_tracker.h"

#include <glog/logging.h>

#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

#include "ring_buffer.hpp"

using namespace cv;

/**
  LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

void LKFeatureTracker::printStats() const {
  LOG(INFO) << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_;
  LOG(INFO) << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_;
  LOG(INFO) << "Avg. Number of matches: " << avg_num_matches_;
  LOG(INFO) << "Avg. Number of good matches: NA";
  LOG(INFO) << "Avg. Number of Inliers: " << avg_num_inliers_;
  LOG(INFO) << "Avg. Inliers ratio: " << avg_inlier_ratio_;
  LOG(INFO) << "Num. of samples: " << num_samples_;
}

LKFeatureTracker::~LKFeatureTracker() {
  printStats();
  cv::destroyWindow(window_name_);
}

/** TODO This is the main tracking function. It takes in the current frame and
 * detects features that correspond to the previous frame.
@param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sequences we provided you by using the Harris
  //   corner detector. Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and
  //   cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  Mat gray_frame , prev_gray_frame;  // gray frame for current image 

  Mat mask = Mat::zeros(frame.size() , frame.type()); // for drawing

  std::vector<Point2f> p0, p1; 

  if (LKFeatureTracker::prev_frame_.empty()){
    cvtColor(frame , gray_frame, COLOR_BGR2GRAY); 
    goodFeaturesToTrack(gray_frame , p0 , 100 ,
                      0.3 , 7 , Mat() , 7 , true , 0.04  );

    LKFeatureTracker::prev_frame_ = gray_frame.clone();  // if u dont clone its passing reference - bad idea
    LKFeatureTracker::prev_corners_ = p0; 
    return; 
  }

  if (frame.empty()){
    LOG(ERROR) << "Input image to LKFeatureTracker::trackFeatures  is empty";
    return;
  }

  cvtColor(frame , gray_frame, COLOR_BGR2GRAY );

  p0 = LKFeatureTracker::prev_corners_ ;
  prev_gray_frame = LKFeatureTracker::prev_frame_ ;

  // creating random colours to plot
  std::vector<Scalar> colors;
    RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r,g,b));
    }

  std::vector<uchar> status;    // to  record the status of points
  std::vector<float> err;       // 
  TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
  calcOpticalFlowPyrLK(prev_gray_frame, gray_frame, p0, p1, status, err, Size(15,15), 2, criteria);

  std::vector<Point2f> good_new;
      for(uint i = 0; i < p0.size(); i++)
      {
          // Select good points
          if(status[i] == 1) {
              good_new.push_back(p1[i]);
              // draw the tracks
              line(mask,p1[i], p0[i], colors[i], 2);
              circle(frame, p1[i], 5, colors[i], -1);
          }
      }
  
      Mat img;
      add(frame, mask, img);
      imshow("Frame", img);
      waitKey(10);

    // show(frame , p0 , p1); 
    //what is the question - implement differential feature tracker - 
    // they provide with tracking features - using Harris corner -- where is it? 
  
    LKFeatureTracker::prev_frame_ = gray_frame.clone();  // if u dont clone its passing reference - bad
    LKFeatureTracker::prev_corners_ = p0; 
  //
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

bool LKFeatureTracker::inlierMaskComputation(const std::vector<Point2f>& pts1,
                                            const std::vector<Point2f>& pts2,
                                            std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  bool mask_computed = true;  // always optimistic...
  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1,
                      pts2,
                      FM_RANSAC,
                      max_dist_from_epi_line_in_px,
                      confidence_prob,
                       *inlier_mask);
  } catch (...) {
    LOG(WARNING) << "Inlier Mask could not be computed, this can happen if there"
                    "are not enough features tracked.";
    mask_computed = false;
  }
  return mask_computed;
}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame,
                            std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //
  //     **** TODO: FILL IN HERE ***
  //
  //     Hint: look at cv::line and cv::cirle functions.
  //     Hint 2: use imshow to display the image
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
