
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        float zw = (*it).z; // world position in m with y facing left from sensor
        if(zw > -1.40){       

            float val = it->x;
            float maxVal = worldSize.height;
            int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
        }
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
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}


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
        //cout << "num of bounding boxes are = " << enclosingBoxes.size()<<endl;

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, std::string imageNumber,bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));
        //cv::Scalar currColor = cv::Scalar(150, 150, 150);
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
    float lineSpacing = 1.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    //cv::namedWindow(windowName, 1);
    //cv::imshow(windowName, topviewImg);
    cv::imwrite("../output/lidarImage"+imageNumber+".jpg", topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for(auto itr = kptMatches.begin(); itr != kptMatches.end(); ++itr)
    {
        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(itr->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(itr->queryIdx);

        if(boundingBox.roi.contains(kpOuterCurr.pt))
        {
            boundingBox.keypoints.push_back(kpOuterCurr);
            boundingBox.kptMatches.push_back(*itr);
        }

    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double dT;
        // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

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
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence
    cout << "camera TTC medDistRatio = " << medDistRatio << endl;
    dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
        // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    double sum_XPrev = 0, sum_XCurr = 0;
    double dT = 1 / frameRate;
    double average_XPrev,average_XCurr,medDistPrev,medDistCurr;
    vector<double> xDistPrev;
    vector<double> xDistCurr;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        minXPrev = minXPrev > it->x ? it->x : minXPrev;
        sum_XPrev += it->x;
        xDistPrev.push_back(it->x);
    }
    average_XPrev =sum_XPrev/ lidarPointsPrev.size();
    std::sort(xDistPrev.begin(), xDistPrev.end());
    long medIndex = floor(xDistPrev.size() / 2.0);
    medDistPrev = xDistPrev.size() % 2 == 0 ? (xDistPrev[medIndex - 1] + xDistPrev[medIndex]) / 2.0 : xDistPrev[medIndex]; 

    cout << "average x _prev = "<< average_XPrev << ", min = " <<minXPrev <<endl;
    cout << "TTC lidar median x prev = "<< medDistPrev<<endl;
    
    // if((average_XPrev > (minXPrev + 0.5)) || ((average_XPrev < (minXPrev - 0.5))))
    // {
    //     minXPrev = average_XPrev;
    // }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        minXCurr = minXCurr > it->x ? it->x : minXCurr;
        sum_XCurr += it->x;
        xDistCurr.push_back(it->x);
    }
    average_XCurr = sum_XCurr/ lidarPointsCurr.size();
    std::sort(xDistCurr.begin(), xDistCurr.end());
    medIndex = floor(xDistCurr.size() / 2.0);
    medDistCurr = xDistCurr.size() % 2 == 0 ? (xDistCurr[medIndex - 1] + xDistCurr[medIndex]) / 2.0 : xDistCurr[medIndex]; 

    cout << "TTC lidar average x curr = "<< average_XCurr << ", min = " <<minXCurr <<endl;
    cout << "TTC lidar median x curr = "<< medDistCurr << ", vel = " <<dT / (medDistPrev - medDistCurr) <<endl;
    // if((average_XCurr > (minXCurr + 0.5)) || ((average_XCurr < (minXCurr - 0.5))))
    // {
    //     minXCurr = average_XCurr;
    // }

    // compute TTC from both measurements
    
    TTC = medDistCurr * dT / (medDistPrev - medDistCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int occu;
    int max_occu = 0;
    int best_box_id;
    float shrinkFactor = 0.10;
    bool bWait = false;
    std::multimap<int, int>  multmap;
    for(auto itr1 = matches.begin(); itr1 != matches.end();++itr1)
    {
        /*get the prev keypoint and current keypoint*/
        cv::KeyPoint prevKp =  prevFrame.keypoints[itr1->queryIdx];
        cv::KeyPoint currentKp =  prevFrame.keypoints[itr1->trainIdx];

        int prevBoxID = -1;
        int currBoxId = -1;
        for(auto it2 = prevFrame.boundingBoxes.begin(); it2!= prevFrame.boundingBoxes.end(); ++it2)
        {
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);
            if(smallerBox.contains(prevKp.pt))
            {
                prevBoxID = it2->boxID;
            }
        }

        for(auto it2 = currFrame.boundingBoxes.begin(); it2!= currFrame.boundingBoxes.end(); ++it2)
        {
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);
            if(smallerBox.contains(currentKp.pt))
            {
                currBoxId = it2->boxID;
            }
        }

        if(currBoxId != -1 && prevBoxID != -1)
        {
            multmap.insert(pair<int,int>(currBoxId,prevBoxID));
        }
    }

    int maxcount = 0;
    int max_Id = -1;
    std::vector<int> prevIDs;
    for(auto mappItr = multmap.begin();mappItr!=multmap.end();++mappItr)
    {
        auto it = multmap.equal_range(mappItr->first); 
        maxcount = 0;
        std::multimap<int, int>  multmap2;
        for(auto itr = it.first; itr != it.second ; ++itr)
        {
            //cout << "multmap2 = " << itr->second <<" , " << itr->first << endl;
            multmap2.insert(pair<int,int>(itr->second,itr->first));
            prevIDs.push_back(itr->second);
        }
        for(auto itr= prevIDs.begin();itr != prevIDs.end(); ++ itr)
        {
            int count = multmap2.count((*itr));
            if(count>maxcount)
            {
                maxcount = count;
                //cout << "max count = "<< maxcount << endl;
                max_Id = (*itr);
            }
        }
        if(max_Id!=-1)
        {
            bbBestMatches.insert(pair<int,int>(max_Id, mappItr->first));
        }
        else
        {
            cout<<"error" << endl;
        }
    }
    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}
