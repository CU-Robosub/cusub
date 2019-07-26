#include <perception_control/path_orient.h>

using namespace std;
using namespace cv;

namespace perception_control
{
    void PathOrient::onInit()
    {
        NODELET_INFO("Path Orient Server Starting Up!");
        orientedWithPath = false;
        nh = &(getMTNodeHandle());
        if ( !nh->getParam("perception_control/path_orient/bang_bang_deadzone", deadZone) ) { ROS_ERROR("Path Orient couldn't locate params"); abort(); }
        nh->getParam("perception_control/path_orient/yaw_setpoint_carrot", yawCarrot);
        yawPub = nh->advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/setpoint",1);
        wayToggleClient = nh->serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        server = new pathServer(*nh, "path_orient", boost::bind(&PathOrient::execute, this, _1), false);
        server->start();
    }

    void PathOrient::execute(const perception_control::PathOrientGoalConstPtr goal)
    {
        NODELET_INFO("Path Orient received request.");
        // Subscribe to darknet and yaw, NOTE once the subscriber object is out of scope the subscription ends
        ros::Subscriber darknetSub = nh->subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &PathOrient::darknetCallback, this);
        ros::Subscriber yawSub = nh->subscribe("cusub_common/motor_controllers/pid/yaw/state", 1, &PathOrient::yawCallback, this);
        ros::Rate r(10);
        while ( ros::ok() )
        {
            if (server->isPreemptRequested() )
            {
                PathOrientResult result;
                result.oriented = false;
                server->setPreempted(result);
                break;
            } else if ( orientedWithPath )
            {
                PathOrientResult result;
                result.oriented = true;
                server->setSucceeded(result);
                break;
            }
            r.sleep();
        }
    }

    /*
        Determines whether the first nonzero pixel from the top of the image is on the left or right side
        Returns 1 for left side
                0 for right side
     */
    bool PathOrient::firstNonzeroPixelLeftSide(Mat& image)
    {
        uint8_t* pixels = image.data;
        int stride = image.step; //in case cols != strides
        for( int i=0; i<image.rows; i++)
        {
            for( int j=0; j<image.cols; j++)
            {
                if( pixels[i * stride + j] )   // pointer access faster than opencv's at<>()
                {
                    if (j < image.cols / 2)
                        return true;
                    else
                        return false;
                }
            }
        }
    }

    void PathOrient::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        NODELET_INFO_THROTTLE(1, "...orienting");
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(bbs->image, sensor_msgs::image_encodings::RGB8);
        Mat cropped, gray, blurred, binary, morphed, morphed2, markers, borders;
        Rect rect(bbs->bounding_boxes[0].xmin, bbs->bounding_boxes[0].ymin,bbs->bounding_boxes[0].xmax-bbs->bounding_boxes[0].xmin,bbs->bounding_boxes[0].ymax-bbs->bounding_boxes[0].ymin);
        cropped = cv_ptr->image(rect);
        cvtColor(cropped, gray, COLOR_BGR2GRAY);
        blur(gray, blurred, Size(3,3));
        threshold(blurred, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);

        // TODO COUNT NONZERO
        // use the percentage of nonzero pixels in the image to determine the num iterations of dilating / eroding
        // int nonzero = countNonZero(binary);
        // cout << "Area: " << binary.cols * binary.rows << "\t\tNonzero: " << nonzero << endl;
        int num_iters = 2; // 2 is roughly 25% of the image, 4 is roughly 33% of the image
        if( countNonZero(binary) > ( binary.total() / 2) )
        {
            dilate(binary, morphed2, Mat(), Point(-1,-1), num_iters);
            bitwise_not(morphed2, morphed);
        } else {
            erode(binary, morphed, Mat(), Point(-1,-1), num_iters);
        }
        cv::connectedComponents(morphed, markers);
        double min, max;
        minMaxLoc(markers, &min, &max);
        int count;
        int maxCount = 0;
        int maxMarker;
        for( int i=1; i<=max; i++)      // find the largest connected component
        {
            compare(markers, Scalar(i), borders, cv::CMP_EQ);
            count = countNonZero(borders);
            if (count > maxCount)
            {
                maxCount = count;
                maxMarker = i;
            }
        }
        compare(markers, Scalar(maxMarker), borders, cv::CMP_EQ);

        // split image so that we only look at one half of the image and ideally one straight segment of the path marker
        // assume that the path marker segment that's more forward (higher in our image) is the one we should align with
        // split according to aspect ratio
        Mat split_image;
        if (borders.cols > borders.rows) // wide image, split so we have either the right or left side
        {   
            if ( firstNonzeroPixelLeftSide(borders) ) // left side
            {
                Rect rect2(0,0, borders.cols / 2, borders.rows);
                split_image = borders(rect2);
            } else {                                  // right side
                Rect rect2(borders.cols / 2, 0, borders.cols / 2, borders.rows);
                split_image = borders(rect2);
            }
        } else {
            Rect rect2(0,0, borders.cols, borders.rows / 2); // use upper half of image
            split_image = borders(rect2);
        }

        vector<Vec4i> lines;
        HoughLinesP(split_image, lines, 1, CV_PI/180, 10, split_image.rows / 3, 0);
        if ( lines.size() < 1)
        {
            NODELET_WARN_THROTTLE(2, "No lines detected in hough algorithm"); // IF NOT WORKING, GO TO "TODO COUNT NONZERO"
            // imshow("cropped", cropped);
            // imshow("blurred", blurred);
            // imshow("binary", binary);
            // imshow("morphed", morphed);
            // waitKey(0);
            // imshow("cropped", cropped);
            return;
        }
        // NODELET_INFO("Number of lines: %d", (int) lines.size());
        Point bottom_pt = Point(0,0);
        Point top_pt = Point(0,split_image.rows);
        // for( size_t i = 0; i < lines.size(); i++ ) // select the longest line
        // {
        //     Vec4i l = lines[i];
        //     line(color, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255,0,0), 3);
        //     if(l[1] > bottom_pt.y)
        //         bottom_pt = Point(l[0],l[1]);
        //     else if(l[1] < top_pt.y)
        //         top_pt = Point(l[0],l[1]);
        //     if(l[3] < top_pt.y)
        //         top_pt = Point(l[2],l[3]);
        //     if(l[3] > bottom_pt.y)
        //         bottom_pt = Point(l[2],l[3]);
        // }
        for( size_t i = 0; i < lines.size(); i++ ) // select the longest line
        {
            Vec4i l = lines[i];
            
            if(l[1] >= bottom_pt.y && l[3] <= top_pt.y)
            {
                bottom_pt = Point(l[0],l[1]);
                top_pt = Point(l[2],l[3]);
            } else if( l[1] <= top_pt.y && l[3] >= bottom_pt.y )
            {
                top_pt = Point(l[0],l[1]);
                bottom_pt = Point(l[2],l[3]);
            }
        }
        // line(color, top_pt, bottom_pt, Scalar(255,0,0), 3);
        // circle(color, top_pt, 3, Scalar(0,0,255), -1);
        // circle(color, bottom_pt, 3, Scalar(0,0,255), -1);
        // NODELET_INFO("Showing colored image.");
        //imshow("colored", color);
        //waitKey(0);

        float slope = ( (float) bottom_pt.y - top_pt.y) / ( (float) bottom_pt.x - top_pt.x);
        // cout << slope << endl;
        std_msgs::Float64 msg;
        if (abs(slope) >= deadZone)
        {
            ROS_INFO("Path Orient Freezing");
            orientedWithPath = true;
            msg.data = yawState;
        } else {
            if ( slope > 0)
            {
                msg.data = yawState + yawCarrot;
            } else { // negative slope
                msg.data = yawState - yawCarrot;
            }
        }
        yawPub.publish(msg);
        // Add logic to determine which path marker we want to use to orient ourselves, presumably the closer one OR 

        /*
            Here's the logic, assume that when we're on the other side of the bouys, we'll only see one path marker, OR we choose the one that's closer...
            I could add that logic to the visual servoing, if there's two of one object just choose the closer...
            OKAY TODO: if there's more than one of a class, use the one that's closer to our target pixel
            That means the end of Jiangshi & triange buoy should be to go to the other side of the buoy!
         */
    }
    void PathOrient::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }
}
PLUGINLIB_EXPORT_CLASS(perception_control::PathOrient, nodelet::Nodelet);