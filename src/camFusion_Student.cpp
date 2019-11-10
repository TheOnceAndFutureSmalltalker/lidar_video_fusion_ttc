
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // find all points within both bounding boxes
    // and accumulate a sum of the distances between corresponding keypoints
    vector<cv::DMatch> bboxMatches;
    double mean_dist = 0.0;
    for(auto match_itr=kptMatches.begin(); match_itr != kptMatches.end(); ++match_itr)
    {
        cv::KeyPoint prevKeyPoint = kptsCurr[match_itr->queryIdx];
        cv::KeyPoint currKeyPoint = kptsCurr[match_itr->trainIdx];
        if(boundingBox.roi.contains(currKeyPoint.pt) && boundingBox.roi.contains(prevKeyPoint.pt))
        {
            bboxMatches.push_back(*match_itr);
            mean_dist += cv::norm(currKeyPoint.pt - prevKeyPoint.pt);
        }
    }
    // calculate mean distance between corresponding keypoints
    mean_dist = mean_dist / bboxMatches.size();
    // only keep matches whose keypoint distance is within some distance of mean
    double min_dist = mean_dist * 0.6;
    double max_dist = mean_dist * 1.4;
    for(auto match_itr=bboxMatches.begin(); match_itr != bboxMatches.end(); ++match_itr)
    {
        cv::KeyPoint prevKeyPoint = kptsCurr[match_itr->queryIdx];
        cv::KeyPoint currKeyPoint = kptsCurr[match_itr->trainIdx];
        double dist = cv::norm(currKeyPoint.pt - prevKeyPoint.pt);
        //cout << "mean: " << mean_dist << "  dist: " << dist << endl;
        if(dist > min_dist && dist < max_dist)
        {
            boundingBox.kptMatches.push_back(*match_itr);
        }
    } 
    //cout << "total matches: " << kptMatches.size() << "  bbox matches: " << bboxMatches.size() << "  kept: " << boundingBox.kptMatches.size() << endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    //cout << "Num Keypopint Matches " << kptMatches.size() << endl;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 5.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            //cout << "distCurr=" << distCurr << "  distPrev=" << distPrev << endl;
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        //cout << "RETURNING FROM TTCCAMERA!!!" << endl;
        return;
    }

    // from "the back of the book!"
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    //std::cout << "medDistRatio = " << medDistRatio << std::endl;
    //std::cout << "meanDistRatio = " << meanDistRatio << std::endl;

    double dT = 1.0 / frameRate;
    //TTC = -dT / (1.0 - medDistRatio);
    TTC = -dT / (1.0 - meanDistRatio);
}

void get_stats(std::vector<LidarPoint> &lidarPoints, double &mean, double &std_dev)
{
    vector<double> distances;
    double sum_distances = 0;
    for(auto lp : lidarPoints)
    {
        sum_distances += lp.x;
    }
    mean = sum_distances / lidarPoints.size();
    double sum_sqrd_diff = 0;
    for(auto lp : lidarPoints)
    {
        sum_sqrd_diff += (mean - lp.x) * (mean - lp.x);
    }
    double variance = sum_sqrd_diff / lidarPoints.size();
    std_dev = sqrt(variance);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = (1.0 / frameRate); // time between two measurements in seconds
    double mean, meanXCurr, meanXPrev;
    double std_dev;

    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    get_stats(lidarPointsPrev, meanXPrev, std_dev);
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        // only consider points within 3 std_dev of mean
        if(abs(it->x-meanXPrev) < 3*std_dev)
        {
            minXPrev = minXPrev>it->x ? it->x : minXPrev;
        }
        else
        {
            //cout << "outlier point = " << it->x << endl;
        }
    }
    //cout << "prev mean=" << meanXPrev << "  std_dev=" << std_dev << "  min=" << minXPrev << endl;

    get_stats(lidarPointsCurr, meanXCurr, std_dev);
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        // only consider points within 3 std_dev of mean
        if(abs(it->x-meanXCurr) < 3*std_dev)
        {
            minXCurr = minXCurr>it->x ? it->x : minXCurr;
        }
        else
        {
            //cout << "outlier point = " << it->x << endl;
        }       
    }
    //cout << "curr mean=" << meanXCurr << "  std_dev=" << std_dev << "  min=" << minXCurr << endl;
    

    // compute TTC from both measurements
  //  TTC = minXCurr * dT / (minXPrev-minXCurr); 
    TTC = meanXCurr * dT / (meanXPrev-meanXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    
    // this map contains an entry for each time that a box in current frame shares
    // a keypoint with a box in previous frame
    multimap<int, int> box_matches;

    // the following logic assumes bounding boxes within a single frame can overlap
    // for each keypoint appearing in both image frames
    for (auto match_itr = matches.begin(); match_itr != matches.end(); ++match_itr)
    {
        // find all boxes in previous frame containing the keypoint
        cv::KeyPoint prevKeyPoint = prevFrame.keypoints[match_itr->queryIdx];
        vector<int> prevBoxIDs;
        for(auto bbx_itr = prevFrame.boundingBoxes.begin(); bbx_itr != prevFrame.boundingBoxes.end(); ++bbx_itr)
        {
            if(bbx_itr->roi.contains(prevKeyPoint.pt))
            {
                prevBoxIDs.push_back(bbx_itr->boxID);
            }
        }

        // find all boxes in current frame containing the keypoint
        cv::KeyPoint curKeyPoint = currFrame.keypoints[match_itr->trainIdx];
        vector<int> curBoxIDs;
        for(auto bbx_itr = currFrame.boundingBoxes.begin(); bbx_itr != currFrame.boundingBoxes.end(); ++bbx_itr)
        {
            if(bbx_itr->roi.contains(curKeyPoint.pt))
            {
                curBoxIDs.push_back(bbx_itr->boxID);
            }
        }

        // all containing boxes in current frame can potentially be paired
        // with all containing boxes in previous frame
        for(auto curBoxID : curBoxIDs)
        {
            for(auto prevBoxID : prevBoxIDs )
            {
                box_matches.insert({curBoxID, prevBoxID});
            }
        }
    }

    int num_prev_boxes = prevFrame.boundingBoxes.size();

    // for bounding box in the current frame
    for(auto box_itr = currFrame.boundingBoxes.begin(); box_itr != currFrame.boundingBoxes.end(); ++box_itr)
    {
        // get range of predetermined keypoint matches for this bounding box
        auto range = box_matches.equal_range(box_itr->boxID);

        // now count how many times each given value appears in this range
        // and the one that appears most is the boxID of the previous frame
        // that best matches (prevFrame boxIDs are the values in box_matches!)
        vector<int> histogram(num_prev_boxes, 0);
        for(auto match_itr = range.first; match_itr != range.second; ++match_itr)
        {
            histogram[match_itr->second] += 1;
        }
        int bestPrevBoxID = max_element(histogram.begin(), histogram.end()) - histogram.begin();

        // add this pair to result
        bbBestMatches.insert({bestPrevBoxID, box_itr->boxID});
    }
}
