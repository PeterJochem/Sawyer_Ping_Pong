## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Describe class here
class videoStream():

    def __init__(self):
        # Create a pipeline
        pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Streaming loop
        try:
            while True:
                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image
                

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
            
                hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

                # define range of yellow color in HSV
                lower_yellow_range = np.array( [20, 22, 115] )
                upper_yellow_range = np.array( [40, 150, 254] )

                # Threshold the HSV image to get only yellow colors
                color_image = cv2.inRange(hsv, lower_yellow_range, upper_yellow_range)
        
                kernel = kernel = np.ones( (5,5), np.uint8)
                color_image = cv2.morphologyEx(color_image, cv2.MORPH_OPEN, kernel)
                ignore2, contours, ignore1 = cv2.findContours(color_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
                frame = color_image
                #res = cv2.bitwise_and(frame, frame, mask = mask)
                    # cv2.imshow('frame',frame)
                    # cv2.imshow('res', res)
        
                # Perform MORPHOLOICAL OPENING!!!!
                # PERFORM MORPHOLICAL OPENING!!!!

                # Blank image - just draw the largest contour
                # cv2.drawContours(color_image, contours, -1, (255,255,0), 3)
        
                color_image = np.zeros( ( len(color_image), len(color_image[0]) )  )

                indexOfLargest = -1
                largestContour = -1

                for i in range( len(contours) ):
                    if ( (cv2.contourArea(contours[i], False) ) > largestContour):
                        largestContour = cv2.contourArea(contours[i], False)
                        indexOfLargest = i
        
                if (indexOfLargest != -1):
    
                    #print(contours)
                    real_contour = contours[indexOfLargest]
                    color_image = cv2.drawContours(color_image, real_contour, -1, (255,255,0), 3)
                    kernel = kernel = np.ones((5,5), np.uint8) 
                else:
                    # Draw the prior contour?
                    pass 
                
                
                intrinsics = frames.profile.as_video_stream_profile().intrinsics
                # depth = depth_image.get_distance(50, 50)
                depth = aligned_depth_frame.get_distance( int(640.0/2.0), int(480.0/2.0) )

                # What units??
                print(depth)
                coord = rs.rs2_deproject_pixel_to_point(intrinsics, [ int(640.0/2.0), int(480.0/2.0) ], depth)  
                print("")
                print("")
                print(coord)
                print("")
                print("")


                #color_image = cv.dilate(color_image, kernel,iterations = 1)
                #color_image = cv2.morphologyEx(color_image, cv2.MORPH_OPEN, kernel)
                #color_image = cv2.morphologyEx(color_image, cv2.MORPH_CLOSE, kernel)
                # print(indexOfLargest)


                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 255
                depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) #depth image is 1 channel, color is 3 channels
                # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

                # Render images
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
                # Convert the image to HSV space
                color_image_copy = color_image # color_image.copy()  
                # Why does copying the image not slow it down a lot?

                # Filter on the orange hue
                    # Orange = [] - []
                # Traverse the image and black/Null out the bad pixels

                # Present the image
                # images = np.hstack( (color_image, depth_colormap) )
                cv2.namedWindow('Track Tennis Ball', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Track Tennis Ball', color_image)
                key = cv2.waitKey(1)

                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break

        finally:
            pipeline.stop()



# Main
#myStream = videoStream()


