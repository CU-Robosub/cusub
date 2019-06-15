/*
    The start gate localizer that uses the watershed algorithm
    along with a pnp solve to localize the gate.
 */

#include <localizer/start_gate_watershed.h>

namespace pose_generator
{   
    void StartGateWatershed::sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs)
    {
        // We want [left_leg, middle_leg, right_leg]
        ;
    }
    bool StartGateWatershed::getPoints(Mat& img, vector<Point>& points)
    {
        Mat gray, binary, eroded;
        vector<Vec4i> lines;

        cvtColor(img, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);
        erode(binary, eroded, Mat());
        HoughLinesP(eroded, lines, 1, CV_PI/180, img.rows/4,img.rows/4,10);
        if(lines.empty()) {return false;}
        
        Point bottom_pt = Point(0,0);
        Point top_pt = Point(0,img.rows);
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            if(l[1] > bottom_pt.y)
                bottom_pt = Point(l[0],l[1]);
            if(l[3] < top_pt.y)
                top_pt = Point(l[2],l[3]);
        }
        points.push_back(bottom_pt);
        points.push_back(top_pt);
        return true;
    }
    bool StartGateWatershed::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::Pose& pose,
        string& class_name
    ){
        if(bbs.size() != 2) {return false;} // to be adjusted for 3 leg case
        sortLegs(bbs);
        ROS_INFO("Localizing the gate!");
        // Double check that its RGB8 not BGR8
        Rect left_rect(bbs[0].xmin, bbs[0].ymin,bbs[0].xmax-bbs[0].xmin,bbs[0].ymax-bbs[0].ymin);
        Rect right_rect(bbs[1].xmin, bbs[1].ymin,bbs[1].xmax-bbs[1].xmin,bbs[1].ymax-bbs[1].ymin);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        Mat left_pole = cv_ptr->image(left_rect);
        Mat right_pole = cv_ptr->image(right_rect);
        vector<Point> points;
        if(!getPoints(left_pole, points)) {return false;}
        if(!getPoints(right_pole, points)) {return false;}

        // Transform Points back to main image
        points[0].x += bbs[0].xmin;
        points[0].y += bbs[0].ymin;
        points[1].x += bbs[0].xmin;
        points[1].y += bbs[0].ymin;
        points[2].x += bbs[1].xmin;
        points[2].y += bbs[1].ymin;
        points[3].x += bbs[1].xmin;
        points[3].y += bbs[1].ymin;

        circle(cv_ptr->image, points[0],3,Scalar(0,0,255),3);
        circle(cv_ptr->image, points[1],3,Scalar(0,0,255),3);
        circle(cv_ptr->image, points[2],3,Scalar(0,0,255),3);
        circle(cv_ptr->image, points[3],3,Scalar(0,0,255),3);
        imshow("Display Window", cv_ptr->image);
        waitKey(0);

        // Let's display on the big image...
        // PNP solve this bitch!

        
        return true;
    }
    StartGateWatershed::StartGateWatershed()
    {
        ROS_INFO("Initializing StartGate Watershed!");
    }
}

void draw_on_image(Mat& image)
{   
    // Column then row
    // loop through cols and rows (.cols, .rows)
    for(int i=0; i < image.rows; i += 50)
    {
        line(image, Point(0,i), Point(image.cols,i), Scalar(0,0,255));
    }
    for(int j=0; j < image.cols; j += 50)
    {
        line(image, Point(j,0), Point(j,image.rows), Scalar(0,0,255));
    }
}

int main(int argc, char **argv)
{
    cout << "Starting Watershed Testing" <<endl;
    pose_generator::StartGateWatershed sgw;

    string image_name0 = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/frame0000.jpg";
    string image_name1 = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/T0_10-27-18-image0_0137.jpg";
    string image_name2 = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/T0_10-27-18-image0_0160.jpg";
    Mat image0 = imread(image_name0, CV_LOAD_IMAGE_COLOR);

    darknet_ros_msgs::BoundingBox image0_left;
    darknet_ros_msgs::BoundingBox image0_right;
    image0_left.Class = "start_gate_pole";
    image0_left.probability = 1.0;
    image0_left.xmin = 390;
    image0_left.ymin = 160;
    image0_left.xmax = 420;
    image0_left.ymax = 300;
    image0_right.Class = "start_gate_pole";
    image0_right.probability = 1.0;
    image0_right.xmin = 650;
    image0_right.ymin = 155;
    image0_right.xmax = 680;
    image0_right.ymax = 310;
    vector<darknet_ros_msgs::BoundingBox> bbs;
    bbs.push_back(image0_left);
    bbs.push_back(image0_right);

    std_msgs::Header header;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image0);
    sensor_msgs::Image test_image0;
    img_bridge.toImageMsg(test_image0);

    geometry_msgs::Pose pose;
    string class_id;
    if(sgw.generatePose(test_image0, bbs, pose, class_id))
    {
        cout << "localized pose!" <<endl;
    }

    // line(image0, Point(0,10), Point(752,10), Scalar(0,0,255));
    // draw_on_image(image0);
    // rectangle(image0, Point(390,160), Point(420, 300), Scalar(0,0,255));
    // rectangle(image0, Point(650,155), Point(680, 310), Scalar(0,0,255));
    // namedWindow("Display Window", WINDOW_AUTOSIZE);
    // imshow("Display Window", image0);
    // waitKey(0);
    
    // really load the images & display them
    // pass them into generate pose
    return 0;
}