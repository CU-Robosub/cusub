/*
    The jiangshi localizer that uses the watershed algorithm
    along with a pnp solve to localize the jiangshi buoy.
 */

#include <localizer/jiangshi_watershed.h>
#include <opencv2/highgui/highgui.hpp>

namespace pose_generator
{  
    bool JiangshiWatershed::getPoints(Mat& img, int border_size, vector<Point2f>& points)
    {
        cv::imshow("Img", img);
        cv::waitKey(0);

        Mat trimmed, bordered, gray, binary, eroded, dilated, markers, borders, jiangshi, corners, approxDP;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        vector<Point2f> corners_float;
        vector<Point> corners_int;

        // cout << img.cols << endl;
        // cout << img.rows << endl;
        // int dilation = (int) img.cols / 20;
        // Rect rect (border_size, border_size, img.cols - 2*border_size, img.rows-2*border_size);
        // trimmed = img(rect);
        cv::Mat jiangshi_center(cv::Size(img.cols - 2*border_size, img.rows-2*border_size), CV_8U, Scalar(0));
        int top_pixel_x = (int) jiangshi_center.cols / 4;
        int top_pixel_y = (int) jiangshi_center.rows / 4;
        int width = (int) jiangshi_center.cols / 2;
        int length = (int) jiangshi_center.rows / 2;
        Rect rect (top_pixel_x,top_pixel_y, width, length);
        cv::rectangle(jiangshi_center, rect, cv::Scalar(255), -1);
        cv::copyMakeBorder(jiangshi_center, bordered, border_size, border_size,border_size,border_size, cv::BORDER_CONSTANT, Scalar(1,1,1));
        cv::connectedComponents(bordered, markers);
        cv::watershed(img, markers);
        cv::compare(markers, Scalar(2), borders, cv::CMP_EQ);
        cv::findContours(borders, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        double epsilon = 0.1 * cv::arcLength(contours[0], true);
        cv::approxPolyDP(contours[0], approxDP, epsilon, true);
        cout << approxDP << endl;
        cout << approxDP.at<int>(0,0) << endl;
        cout << approxDP.size() << endl;
        cv::circle(img, Point(approxDP.at<int>(0,0), approxDP.at<int>(0,1)), 1, cv::Scalar(255,255,255));
        cv::circle(img, Point(approxDP.at<int>(1,0), approxDP.at<int>(1,1)), 1, cv::Scalar(255,255,255));
        cv::circle(img, Point(approxDP.at<int>(2,0), approxDP.at<int>(2,1)), 1, cv::Scalar(255,255,255));
        cv::circle(img, Point(approxDP.at<int>(3,0), approxDP.at<int>(3,1)), 1, cv::Scalar(255,255,255));
        cv::imshow("contours", img);
        cv::waitKey(0);
        return true;

        // cv::imshow("borders", borders);
        // cv::waitKey(0);

        // cv::imshow("jiangshi_center", jiangshi_center);
        // waitKey(0);
        // return false;
        // initialize an image with zeros, make a rectangle of 2's
        // cv::cvtColor(trimmed, gray, COLOR_BGR2GRAY);
        // cv::threshold(gray, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);
        // cv::erode(binary, eroded, Mat());;
        // cv::dilate(eroded,dilated, Mat(),Point(-1,1), dilation);
        
        
        

        // int dilation = (int) img.cols / 20;
        // Rect rect (border_size, border_size, img.cols - 2*border_size, img.rows-2*border_size);
        // trimmed = img(rect);
        // cv::cvtColor(trimmed, gray, COLOR_BGR2GRAY);
        // cv::threshold(gray, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);
        // cv::erode(binary, eroded, Mat());
        // cv::dilate(eroded,dilated, Mat(),Point(-1,1), dilation);
        // cv::copyMakeBorder(dilated, bordered, border_size, border_size,border_size,border_size, cv::BORDER_CONSTANT, Scalar(1,1,1));
        // cv::connectedComponents(bordered, markers);
        // cv::watershed(img, markers);
        // cv::compare(markers, Scalar(2), borders, cv::CMP_EQ);
        // cv::imshow("borders", borders);
        // cv::waitKey(0);
        // cv::findContours(borders, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        // double epsilon = 0.1 * cv::arcLength(contours[0], true);
        // cv::approxPolyDP(contours[0], approxDP, epsilon, true);
        // cout << approxDP << endl;
        // cout << approxDP.at<int>(0,0) << endl;
        // cout << approxDP.size() << endl;
        // cv::circle(img, Point(approxDP.at<int>(0,0), approxDP.at<int>(0,1)), 1, cv::Scalar(255,255,255));
        // cv::circle(img, Point(approxDP.at<int>(1,0), approxDP.at<int>(1,1)), 1, cv::Scalar(255,255,255));
        // cv::circle(img, Point(approxDP.at<int>(2,0), approxDP.at<int>(2,1)), 1, cv::Scalar(255,255,255));
        // cv::circle(img, Point(approxDP.at<int>(3,0), approxDP.at<int>(3,1)), 1, cv::Scalar(255,255,255));
        // cv::imshow("contours", img);
        // cv::waitKey(0);
        // return true;
    }
    bool JiangshiWatershed::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::Pose& pose,
        string& class_name
        ){
            if( bbs.size() != 1 ) { return false; }
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

            // Trim image to jiangshi
            int border_size = 10; // add room for a border that we'll say is part of the 'not bouy' class
            Rect rect (bbs[0].xmin-border_size, bbs[0].ymin-border_size,bbs[0].xmax-bbs[0].xmin+2*border_size,bbs[0].ymax-bbs[0].ymin+2*border_size);
            Mat img = cv_ptr->image(rect);

            // Transform Points back to main image
            vector<Point2f> img_points;
            if(!getPoints(img,border_size, img_points)) {return false;}
            img_points[0].x += bbs[0].xmin;
            img_points[0].y += bbs[0].ymin;
            img_points[1].x += bbs[0].xmin;
            img_points[1].y += bbs[0].ymin;

            // // Get pose from image points using a solvepnp
            // getPoseFromPoints(truth_pts, img_points, pose); // inherited
            // class_name = "jiangshi";
            return true;
        }
}


// There are probably some bag files that I can download of Jiangshi