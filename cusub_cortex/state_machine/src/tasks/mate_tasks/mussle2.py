import numpy as np
import argparse
import cv2

class MusscleTask:
    def __init__(self,img):

        self.initial_img = np.load(img)

        self.rectangles = self.find_rect(self.initial_img)

        self.blob(self.initial_img)

        self.count_musscles(self.initial_img)

        self.run(self.initial_img,self.rectangles)

        



    
    def find_rect(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret , thrash = cv2.threshold(gray, 240 , 255, cv2.CHAIN_APPROX_NONE)
        _, contours, _= cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        rectangles = []

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
            if len(approx) == 4 :
                rectangles.append(approx)

        rects = []
        for rect in rectangles:
            rects.append(np.float32([rect[0][0],rect[1][0],rect[3][0],rect[2][0]]))
        return rects

    def blob(self,image):
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(gray)
        print(keypoints)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # return self.bridge.cv2_to_imgmsg(im_with_keypoints)
        # Show keypoints
        cv2.imshow("Blob", im_with_keypoints)
        cv2.waitKey(0)



    def count_musscles(self,image):
        output = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
              param1=15,
              param2=40,
              minRadius=10,
              maxRadius=0)
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int") 
            counter = 1
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (255, 0, 0), 4)
                cv2.putText(output,str(counter),(x-r,y+r),cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 0), 2)
                counter += 1

        cv2.imshow("Hough", np.hstack([image, output]))
        cv2.waitKey(0)
        # return self.bridge.cv2_to_imgmsg(output)

    def run(self,image,rectangles):
        pts2 = np.float32([[0,0],[600,0],[0,600],[600,600]])
        for rect in rectangles:
            pts1 = rect
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(image,M,(600,600))

            self.count_musscles(dst)
            self.blob(dst)


    


if __name__ == "__main__":
    mt = MusscleTask("mussle.npy")