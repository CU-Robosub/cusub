/*
    The start gate localizer that uses the Hough algorithm
    along with a pnp solve to localize the gate.
 */

#include <localizer/start_gate_hough.h>

namespace pose_generator
{   
    StartGateHough::StartGateHough()
    {
        ros::NodeHandle nh;
        small_leg_side_pub = nh.advertise<std_msgs::Bool>("cusub_perception/start_gate/small_pole_left_side",1);
        if ( !nh.getParam("localizer/hough/three_legs", three_legs) ) { ROS_ERROR("Startgate couldn't locate params"); abort(); }
        if(!three_legs) { ROS_WARN("Localizing gate with 2 legs."); }
        else { ROS_INFO("Localizing gate with 3 legs."); }
    }

    /*
        Publishes middle leg gate side
        Darknet boxes have already been sorted to:
        [ left, middle, right ]
     */
    void StartGateHough::publishLegSide(vector<darknet_ros_msgs::BoundingBox>& bbs)
    {
        std_msgs::Bool left;
        left.data = abs(bbs[0].xmin - bbs[1].xmin) < abs(bbs[1].xmin - bbs[2].xmin);
        small_leg_side_pub.publish(left);
        if (left.data)
            cout << "publishing left" << endl;
        else
            cout << "publishing right" << endl;
    }

    /*
        Sorts 3 start gate bounding boxes in a frame
        [ left, middle, right ]
     */
    void StartGateHough::sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs)
    {
        // Sort using lambda operator
        sort(bbs.begin(), bbs.end(), []( const darknet_ros_msgs::BoundingBox& b1, const darknet_ros_msgs::BoundingBox& b2 )
            {
                return b1.xmin < b2.xmin;
            });
    }

    /*
        Finds the top and bottom points of the leg using the hough lines algorithm
     */
    bool StartGateHough::getPoints(Mat& img, vector<Point2f>& points)
    {
        Mat gray, binary, eroded;
        vector<Vec4i> lines;

        cvtColor(img, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);
        erode(binary, eroded, Mat());
        HoughLinesP(eroded, lines, 1, CV_PI/180, img.rows/4,img.rows/4,3);
        if(lines.empty()) {return false;}
        
        // Choose highest and lowest points out of all lines
        Point bottom_pt = Point2f(0,0);
        Point top_pt = Point2f(0,img.rows);
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            if(l[1] > bottom_pt.y)
                bottom_pt = Point2f(l[0],l[1]);
            if(l[3] < top_pt.y)
                top_pt = Point2f(l[2],l[3]);
        }
        points.push_back(bottom_pt);
        points.push_back(top_pt);
        return true;
    }

    bool StartGateHough::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        vector<localizer::Detection>& detections
    ){

        localizer::Detection det;
        det.class_id = "start_gate";
        det.pose.header.stamp = image.header.stamp;
        det.pose.header.frame_id = image.header.frame_id;

        if ( three_legs && (bbs.size() < 2 || bbs.size() > 3) ) {return false;}
        if ( !three_legs && bbs.size() != 2) {return false;}

        sortBoxes(bbs);
        // Double check that its RGB8 not BGR8
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

        // Trim each leg out of image
        Rect left_rect(bbs[0].xmin, bbs[0].ymin,bbs[0].xmax-bbs[0].xmin,bbs[0].ymax-bbs[0].ymin);
        Rect right_rect(bbs.back().xmin, bbs.back().ymin,bbs.back().xmax-bbs.back().xmin,bbs.back().ymax-bbs.back().ymin);
        Mat left_pole = cv_ptr->image(left_rect);
        Mat right_pole = cv_ptr->image(right_rect);

        // Find gate's corner points
        vector<Point2f> img_points;
        if(!getPoints(left_pole, img_points)) {return false;}
        if(!getPoints(right_pole, img_points)) {return false;}

        // Transform Points back to main image
        img_points[0].x += bbs[0].xmin;
        img_points[0].y += bbs[0].ymin;
        img_points[1].x += bbs[0].xmin;
        img_points[1].y += bbs[0].ymin;
        img_points[2].x += bbs.back().xmin;
        img_points[2].y += bbs.back().ymin;
        img_points[3].x += bbs.back().xmin;
        img_points[3].y += bbs.back().ymin;

        // Get pose from image points using a solvepnp
        getPoseFromPoints(gate_truth_pts, img_points, det.pose.pose); // inherited
        if (three_legs && bbs.size() == 3) { publishLegSide(bbs); }

        detections.push_back(det);

        return true;

    }
}
