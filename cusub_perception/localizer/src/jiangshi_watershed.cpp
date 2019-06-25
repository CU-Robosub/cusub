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
        int dilation = (int) img.cols / 20;
        Rect rect (border_size, border_size, img.cols - 2*border_size, img.rows-2*border_size);
        trimmed = img(rect);
        cv::cvtColor(trimmed, gray, COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 0, 2, THRESH_BINARY_INV + THRESH_OTSU);
        cv::erode(binary, eroded, Mat());
        cv::dilate(eroded,dilated, Mat(),Point(-1,1), dilation);
        cv::copyMakeBorder(dilated, bordered, border_size, border_size,border_size,border_size, cv::BORDER_CONSTANT, Scalar(1,1,1));
        cv::connectedComponents(bordered, markers);
        cv::watershed(img, markers);
        cv::compare(markers, Scalar(2), borders, cv::CMP_EQ);
        cv::imshow("borders", borders);
        cv::waitKey(0);
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

        // cv::circle(img, corners_int[i], 1, Scalar(255,255,255), 3);
        // cv::drawContours(img, approxDP, 0, cv::Scalar(255,255,255), 3);

        // cout << contours[0] << endl;
        // for( int i=0; i<contours.size(); i++)
        // {
        //     cv::Rect r = cv::boundingRect(contours[i]);
        //     cv::rectangle(img, r, Scalar::all(255));
        //     // cv::Point p0(r.x, r.y);
        //     // cv::Point p1(r.x+r.width, r.y + r.height);
        //     // cv::rectangle(img, p0,p1)
        // }
        cv::imshow("contours", img);
        cv::waitKey(0);


        // cv::cornerHarris(borders, corners, 7, 5, 0.05, cv::BORDER_DEFAULT);

        // cv::imshow("corners", corners);
        // cv::waitKey(0);

        // double qualityLevel = 0.01;
        // double minDistance = 10;
        // int blockSize = 3;
        // bool useHarrisDetector = true;
        // double k = 0.04;
        // cv::goodFeaturesToTrack(borders, corners_float, 4, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);
        // cout << corners_float.size() << endl;
        // for(int i=0; i < corners_float.size(); i++)
        // {
        //     corners_int.push_back( cv::Point( (int) corners_float[i].x, (int) corners_float[i].y));
        //     cv::circle(img, corners_int[i], 1, Scalar(255,255,255), 3);
        // }
        

        // cv::imshow("borders", borders);
        // cv::waitKey(0);
        // cv::imshow("corners", img);
        // cv::waitKey(0);

        // cv::RotatedRect rrect = cv::minAreaRect(contours[0]);
        // cv::Point2f* pts;
        // rrect.points(pts);
        // cout << sizeof(pts) / sizeof(pts[0]) << endl;

        // for(int i=0; i<rrect.points; i++)
        // {
        //     cv::Point p;
        //     p[0] = int(rrect.points[i][0]);
        //     p[1] = int(rrect.points[i][1]);
        //     cv::circle(img, p, 1, cv::Scalar(255,255,255), 1);
        // }

        
        // cv::RNG rng(12345);
        // Mat drawing = Mat::zeros( borders.size(), CV_8UC3 );
        // for( int i = 0; i< contours.size(); i++ )
        // {
        //     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //     drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        //     namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        //     cout << "---------------------------------------------------" << endl;
        //     cout << contours[i] << endl;
        //     imshow( "Contours", drawing );
        //     waitKey(0);
        // }
        

        // cout << markers.size() << endl;
        // cout << img.size() << endl;
        
        // cout << markers << endl;
        // cout << borders << endl;
        // cv::imshow("watershed", markers);
        // cv::waitKey(0);
        // cv::imshow("binary", binary);
        // cv::waitKey(0);
        // cv::imshow("eroded", eroded);
        // cv::waitKey(0);
        // cv::imshow("dilated", dilated);
        // cv::waitKey(0);
        // cv::imshow("dilated", dilated2);
        // cv::waitKey(0);
        // cv::imshow("dilated", dilated3);
        // cv::waitKey(0);
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
            int border_size = 20; // add room for a border that we'll say is part of the 'not bouy' class
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