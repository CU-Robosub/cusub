#!/usr/bin/env python
from __future__ import division
"""
This class attempts to find the 4 desired points on the startgate

"""
import numpy as np
import cv2
from classical_cv.pnp_analyzer import PNP_Analyzer
import scipy.stats
import pdb

class Pixel():
    def __init__(self, row, col):
        self.row = row
        self.col = col
    def __str__(self):
        return "("+str(self.row)+","+str(self.col)+")"

class GatePNPAnalyzer(PNP_Analyzer):

    class PixelGaussian():

        # Initialize empty array
        leg_pixels = []

        def __init__(self, img, pixel_list, prob_thresh):
            assert len(pixel_list) > 0 # we must have pixels to start with
            self.img = img
            self.prob_thresh = prob_thresh
            self.add_pixels( pixel_list )

        def _pixel_get_diff(self, pixel):
            """
            Takes the difference between a pixel and the self.avg_leg_pixel
            Pixels in form: [row, col]
            Assumes RGB
            """
            pixel_vec = self.img[pixel.row][pixel.col][:]
            diff = pixel_vec - self.avg_leg_pixel
            return np.linalg.norm(diff)

        def classify(self, pixel):
            # returns a true or false value with whether the pixel belongs to the given class
            diff = self._pixel_get_diff(pixel)
            prob = 1 - self.leg_distribution.cdf(diff)
#            print(prob)
            if prob > self.prob_thresh:
                return True
            else:
                return False

        def add_pixels(self, pixel_list):
            # should be a list of pixels
            # adds pixels and updates the RGB avg, mean, std
            assert type(pixel_list) == list
            assert isinstance(pixel_list[0], Pixel)

            # add the pixels to the leg_pixels
            self.leg_pixels = self.leg_pixels + pixel_list

            b_arr = np.empty([0,0])
            g_arr = np.empty([0,0])
            r_arr = np.empty([0,0])

            # Loop through pixels and populate color numpy arrays
            for pixel in self.leg_pixels:
                pixel_vector = self.img[pixel.row][pixel.col]
#                print pixel_vector
                b_arr = np.append(b_arr, pixel_vector[0])
                g_arr = np.append(g_arr, pixel_vector[1])
                r_arr = np.append(r_arr, pixel_vector[2])

            self.avg_leg_pixel = np.array([np.mean(b_arr),\
                                           np.mean(g_arr),\
                                           np.mean(r_arr)])

#            print(self.avg_leg_pixel)

            pixel_diffs = np.empty([0,0])
            for pixel in self.leg_pixels:
                pixel_diffs = np.append(pixel_diffs, self._pixel_get_diff(pixel))

#            print(pixel_diffs)
            self.leg_distribution = scipy.stats.norm(np.mean(pixel_diffs),\
                                                     np.square(np.std(pixel_diffs)))

            # print("mean: "+ str(np.mean(pixel_diffs)))
            # print("std: "+ str(np.std(pixel_diffs)))
            # for pixel in self.leg_pixels:
            #     self.classify(pixel)

    def __init__(self, img, color_space): # should already by an OpenCV image
        self.img = img
        self.img_height, self.img_width, self.img_channels = img.shape
        self.color_space = color_space


    def color_pixels(self, pixel_list):
        """
        Used mainly as a debugging tool
        """
        for p in pixel_list:
            cv2.circle(self.img, (p.col,p.row), 1, (0,255,0), -1)
        cv2.imshow('image',self.img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def localize_leg(self, xmin, ymin, xmax, ymax):
        # generate pixels from bounding_box
        y_avg = int( (ymin + ymax) / 2)
        x_avg = int( (xmin + xmax) / 2)

#        point = self.get_point( Pixel(y_avg,x_avg), down_dir=True)
#        self.color_pixels([Pixel(y_avg,x_avg)])
#        return

        record_top_pixel = Pixel(self.img_height,0) # we want smaller rows
        record_bottom_pixel = Pixel(0,0) # we want larger rows
        for col in range(xmin, xmax):
            bottom_pixel = self.get_point( Pixel(y_avg,col), down_dir=True)
            top_pixel = self.get_point( Pixel(y_avg,col), down_dir=False)
            # print(isinstance(bottom_pixel, Pixel))
            # print(isinstance(top_pixel, Pixel))
            # pdb.set_trace()
            if top_pixel == None or bottom_pixel == None:
                continue
            else:
                if record_top_pixel.row > top_pixel.row:
                    record_top_pixel = top_pixel
                if record_bottom_pixel.row < bottom_pixel.row:
                    record_bottom_pixel = bottom_pixel

        #self.color_pixels([record_top_pixel, record_bottom_pixel])
        return (record_top_pixel.col, record_top_pixel.row),\
            (record_bottom_pixel.col, record_bottom_pixel.row)

        # cycle through starting pixels
        # calls get_point twice, once for each direction
        # decides if the result is reasonable

    def get_point(self, starting_pixel, down_dir=True):
        # returns the point arrived at while traveling the leg
        # call classify_pixels on the row beneath us
        # count the number of adjacent ones
        # if the number is less than 3 return our current center pixel
        # if it's not center ourselves, add the ones to the pixelGaussian
        # repeat on the next line :)
        r = starting_pixel.row
        c = starting_pixel.col
        """
        [r-1,c-1] [r-1,c  ] [r-1,c+1]
        [r  ,c-1] [r  ,c  ] [r  ,c+1]
        [r+1,c-1] [r+1,c  ] [r+1,c+1]
        """
        init_leg_pixels = [Pixel(r-1,c-1),\
                           Pixel(r-1,c),\
                           Pixel(r-1,c+1),\
                           Pixel(r,c-1),\
                           Pixel(r,c),\
                           Pixel(r,c+1),\
                           Pixel(r+1,c-1),\
                           Pixel(r+1,c),\
                           Pixel(r+1,c+1)]

        pg = self.PixelGaussian(self.img, init_leg_pixels, 0.0)

        num_search_pixels = 10
        center_pixel = starting_pixel
#        print("center_pixel: " + str(center_pixel))
        nonzero_indices = [1,1,1] # arbitrary set to size larger than 3
        path = []
        shifting = 0
        while len(nonzero_indices) > 1:
#            print("#####################")
            if center_pixel.row >= self.img_height: # ERROR
                return None
            prev_center_col = center_pixel.col
            if down_dir: # going down
                center_pixel.row += 1
            else: # going up
                center_pixel.row -= 1
            pixel_arr = self.classify_pixels(pg, center_pixel, num_search_pixels)
            nonzero_indices = list(list(np.nonzero(pixel_arr))[1]) # returns a list of indices that are nonzero
            if len(nonzero_indices) > 7:
#               print("Too many indices")
                break
            elif len(nonzero_indices) == 0:
                break
            center_nonzero_col = nonzero_indices[ int( len(nonzero_indices) / 2) ] # use the middle of the nonzero indices list
            center_pixel.col = center_pixel.col - num_search_pixels + center_nonzero_col # recenter

            # STOP CONDITIONS
            # Don't allow jumping by more than 1 pixel
            if abs(center_pixel.col - prev_center_col) > 1:
                break
            if center_pixel.col != prev_center_col:
                shifting += 1
            else:
                shifting = 0
            if shifting > 2: # don't shift more than 2 times
                break

            tmp = Pixel(center_pixel.row, center_pixel.col)
            path.append(tmp)
            # add the new pixels to the gaussian using add_pixels
            # print nonzero_indices
            # print center_nonzero_col
            # print center_pixel

#        self.color_pixels(path)
        return center_pixel


        # Test to ensure pixel probability working
        # print "##########"
        # test1 = Pixel(r+2,c)
        # test2 = Pixel(r,c+10)
        # print("high prob")
        # pg.classify(test1) # should be high prob
        # print("Low prob")
        # pg.classify(test2) # should be low prob
        # cv2.circle(self.img, (test1.col,test1.row), 1, (255,0,0), -1)
        # cv2.circle(self.img, (test2.col,test2.row), 1, (0,0,255), -1)


    def classify_pixels(self, pg, center_pixel, num_cols_each_side):
        # classifies an array of pixels using the internal pixelGaussian
        # returns an array of the same size with ones for leg classes and zeros for not classes
        # Also return what column numbers each class corresponds to
        row = center_pixel.row
        col = center_pixel.col
        classes = np.empty([1,2*num_cols_each_side + 1])
        for i in range(2 * num_cols_each_side + 1):
            j = i - num_cols_each_side
            pixel = Pixel(row, col + j)
            classes[0,i] = pg.classify(pixel)
        return classes


################### TESTING ########################

def get_center_of_box(xmin, ymin, xmax, ymax):
    """ returns center column then center row of box """
    x_avg = int( (xmin + xmax) / 2 )
    y_avg = int( (ymin + ymax) / 2 )
    return [y_avg, x_avg]

def test1():
    img = cv2.imread('T0_10-27-18-image0_0074.jpg')
    # for i in range(7):
    #     cv2.line(img, (i*100, 0),(i*100, 480), (0,255,0))
    # for j in range(4):
    #     cv2.line(img, (0, j*100),(750, j*100), (0,255,0))
    # cv2.imshow('image',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
#    return

    g = GatePNPAnalyzer(img, "bgr")

    # bounding boxes for 0074
    b1_xmin=225
    b1_ymin=150
    b1_xmax=300
    b1_ymax=300

    b2_xmin=400
    b2_ymin=100
    b2_xmax=500
    b2_ymax=300

    # bounding boxes for 0061
    # b1_xmin=195
    # b1_ymin=191
    # b1_xmax=216
    # b1_ymax=324

    # b2_xmin=407
    # b2_ymin=219
    # b2_xmax=430
    # b2_ymax=352

    g.localize_leg(b1_xmin, b1_ymin, b1_xmax, b1_ymax)
    g.localize_leg(b2_xmin, b2_ymin, b2_xmax, b2_ymax)

    pixel_center_1 = get_center_of_box(b1_xmin, b1_ymin, b1_xmax, b1_ymax)
    pixel_center_2 = get_center_of_box(b2_xmin, b2_ymin, b2_xmax, b2_ymax)

if __name__ == "__main__":
    test1()
