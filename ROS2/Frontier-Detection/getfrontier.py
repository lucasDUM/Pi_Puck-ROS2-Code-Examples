#--------Include modules---------------#
from copy import copy
import rclpy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#from matplotlib import pyplot as plt
#-----------------------------------------------------#

def getfrontier(mapData):
        """
        Extract the position and orientation data from Odometry data
        This will evntually be a proper state estimator but for now it's a filler
        """
        data=mapData.data
        w=mapData.info.width
        h=mapData.info.height
        resolution=mapData.info.resolution
        Xstartx=mapData.info.origin.position.x
        Xstarty=mapData.info.origin.position.y
        
        img = np.zeros((h, w, 1), np.uint8)
        
        for i in range(0,h):
                for j in range(0,w):
                    if data[i*w+j]==100:
                        img[i,j]=0
                    elif data[i*w+j]==0:
                        img[i,j]=255
                    elif data[i*w+j]==-1:
                        img[i,j]=205
         
        #Have a look at the image for testing purposes               
        #imgplot = plt.imshow(img.squeeze(axis=2), cmap='gray', vmin = 0, vmax = 255,interpolation='none')
        #plt.show()
        '''
        inRange(src, lowerb, upperb) defines a threshold for an image
        A pixel is set to 255 if it lies within the boundaries specified otherwise set to 0.  	 This way it returns the thresholded image.
        We can use this to create a mask for the image then do a bitwise and to get the 		theresholded image
        '''
        #Gets occupied sections
        o = cv2.inRange(img,0,1)
        #See https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html
        edges = cv2.Canny(img,0,255)
        #See https://docs.opencv.org/4.5.2/d4/d73/tutorial_py_contours_begin.html
        #contours, hierarcy = findContours(image, mode, method, offset)
        #hiearacy = containing information about the image topology
        contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #image, contours, index of contours (-1 = draw all), colour (255, 255, 255), thickness etc 
        cv2.drawContours(o, contours, -1, (255,255,255), 5)
        #Inverts value of pixels in image 0 for example if pixel value = 255 it will convert that to twos complement i.e. 11111111 and turn that into 00000000 or 0 in 10's complement
        o=cv2.bitwise_not(o)
        #Likewise with not this is just and and 
        #Find possible places where frontiers can exist
        res = cv2.bitwise_and(o,edges) 
        #------------------------------#
        
        frontier=copy(res)
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        all_pts=[]
        if len(contours)>0:
                upto=len(contours)-1
                i=0
                maxx=0
                maxind=0
                
                for i in range(0,len(contours)):
                    cnt = contours[i]
                    #We can find the center of the blob(centroid) using moments in OpenCV
                    # Image Moment is a particular weighted average of image pixel intensities, with the help of which we can find some specific properties of an image, like radius, area, centroid etc. 
                    #To find the centroid of the image, we generally convert it to binary format and then find its center.
                    #So we find the centre of each local contour
                    M = cv2.moments(cnt)
                    #Sometimes if segmentaion doesn't happen percfctly innteh contour you may get a dvision by zero error
                    #X coord of centroid of frontier
                    cx = int(M['m10']/M['m00'])
                    #Y coord of centroid of frontier
                    cy = int(M['m01']/M['m00'])
                    #Convert to real X coordinate
                    xr=cx*resolution+Xstartx
                    #Convert to real Y coordinate
                    yr=cy*resolution+Xstarty
                    #array of points
                    pt=[np.array([xr,yr])]
                    if len(all_pts)>0:
                        #https://numpy.org/doc/stable/reference/generated/numpy.vstack.html#:~:text=vstack-,numpy.,to%20(1%2CN).
                        all_pts=np.vstack([all_pts,pt])
                    else:
                        all_pts=pt
        return all_pts
