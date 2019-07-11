/*
    The jiangshi localizer that uses the watershed algorithm
    along with a pnp solve to localize the jiangshi buoy.
 */

#include <localizer/jiangshi_watershed.h>

namespace pose_generator
{  
    JiangshiWatershed::JiangshiWatershed()
    {
        ros::NodeHandle nh;
        useAspectRatio = false;
        if ( !nh.getParam("localizer/jiangshi_watershed/use_aspect_ratio", useAspectRatio) ) { ROS_ERROR("Jiangshi couldn't locate params"); abort(); }
        if(!useAspectRatio) { ROS_WARN("Jiangshi Watershed not using bb aspect ratio"); }
        else { ROS_INFO("Jiangshi Watershed using bb aspect ratio"); }
    }
    bool JiangshiWatershed::getPoints(Mat& img, int border_size, vector<Point2f>& points)
    {
        Mat bordered, markers, borders, approxDP;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        // Draw rectangle to classify jiangshi points
        if ( (img.cols - 2*border_size < 4) || ( img.rows-2*border_size < 4 ) ) { return false; } // min rectangle width 4 pixels
        cv::Mat jiangshi_center(cv::Size(img.cols - 2*border_size, img.rows-2*border_size), CV_8U, Scalar(0));
        int top_pixel_x = (int) jiangshi_center.cols / 4;
        int top_pixel_y = (int) jiangshi_center.rows / 4;
        int width = (int) jiangshi_center.cols / 2;
        int length = (int) jiangshi_center.rows / 2;
        Rect rect (top_pixel_x,top_pixel_y, width, length);
        cv::rectangle(jiangshi_center, rect, cv::Scalar(255), -1);

        // Draw a border around bounding box to classify water points
        cv::copyMakeBorder(jiangshi_center, bordered, border_size, border_size,border_size,border_size, cv::BORDER_CONSTANT, Scalar(1,1,1));

        // Run watershed
        cv::connectedComponents(bordered, markers);
        cv::watershed(img, markers);

        // Find contours around watershed markers of jiangshi
        cv::compare(markers, Scalar(2), borders, cv::CMP_EQ);
        cv::findContours(borders, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        double epsilon = 0.1 * cv::arcLength(contours[0], true);

        // Polygon fit a rectangle to first contour
        cv::approxPolyDP(contours[0], approxDP, epsilon, true);
        sortPoints(approxDP, points);
        return true;
    }
    
    /*
        Sort the img corner points to match the order of ground truth points
     */
    void JiangshiWatershed::sortPoints(Mat& array, vector<Point2f>& points)
    {
        Point2f top_left, top_right, bot_left, bot_right;
        int row_sum = 0;
        int col_sum = 0;
        for(int i=0; i<array.rows; i++)
        {
            col_sum += array.at<int>(i,0);
            row_sum += array.at<int>(i,1);
        }
        for(int j=0; j<array.rows; j++)
        {
            if (array.at<int>(j,0) > col_sum / 4)       // right side
            {
                if(array.at<int>(j,1) > row_sum / 4)    // bottom right
                    bot_right = Point2f((float)array.at<int>(j,0), (float)array.at<int>(j,1));
                else                                    // top right
                    top_right = Point2f((float)array.at<int>(j,0), (float)array.at<int>(j,1));
            } else {                                    // left side
                if(array.at<int>(j,1) > row_sum / 4)    // bottom left
                    bot_left = Point2f((float)array.at<int>(j,0), (float)array.at<int>(j,1));
                else                                    // top left
                    top_left = Point2f((float)array.at<int>(j,0), (float)array.at<int>(j,1));
            }
        }
        points.push_back(bot_left);
        points.push_back(top_left);
        points.push_back(bot_right);
        points.push_back(top_right);
    }

    /*
        Checks that each bounding box is border_size away from the edge of the image.
     */
    bool JiangshiWatershed::checkBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs, int border_size)
    {
        for( int i=0; i<bbs.size(); i++)
        {
            if( bbs[i].xmin - border_size <= 0 ||
                bbs[i].ymin - border_size <= 0 ||
                bbs[i].xmax + border_size >= 751 ||
                bbs[i].ymax + border_size >= 479)
                { return false; }
        }
        return true;
    }

    geometry_msgs::Quaternion JiangshiWatershed::getOrientationFromAspectRatio(darknet_ros_msgs::BoundingBox bb)
    {
        return geometry_msgs::Quaternion();
    }

    bool JiangshiWatershed::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::Pose& pose,
        string& class_name
        ){
            class_name = "jiangshi";
            if( bbs.size() != 1 ) { return false; }
            int border_size = 10; // add room for a border that we'll say is part of the 'not bouy' class
            if ( !checkBoxes(bbs, border_size) ) { return false; }
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

            // Trim image to jiangshi
            Rect rect (bbs[0].xmin-border_size, bbs[0].ymin-border_size,bbs[0].xmax-bbs[0].xmin+2*border_size,bbs[0].ymax-bbs[0].ymin+2*border_size);
            Mat img = cv_ptr->image(rect);

            // find target points on image
            vector<Point2f> img_points;
            if(!getPoints(img,border_size, img_points)) {return false;}
            // Transform Points back to main image
            img_points[0].x += bbs[0].xmin - border_size;
            img_points[0].y += bbs[0].ymin - border_size;
            img_points[1].x += bbs[0].xmin - border_size;
            img_points[1].y += bbs[0].ymin - border_size;
            img_points[2].x += bbs[0].xmin - border_size;
            img_points[2].y += bbs[0].ymin - border_size;
            img_points[3].x += bbs[0].xmin - border_size;
            img_points[3].y += bbs[0].ymin - border_size;

            // Get pose from image points using a solvepnp
            getPoseFromPoints(truth_pts, img_points, pose); // inherited
            // Adjust Jiangshi Orientation according to aspect ratio of the bounding box
            if ( useAspectRatio ) { pose.orientation = getOrientationFromAspectRatio( bbs[0] ); }
            return true;
        }
}