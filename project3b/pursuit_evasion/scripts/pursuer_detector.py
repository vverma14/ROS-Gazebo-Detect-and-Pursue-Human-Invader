#!/usr/bin/python

import rospy
from std_msgs.msg import Float32 as Float32Msg

from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np


class PursuerDetector():
    def __init__(self, threshold=0.5, height_thresh=225):
        self.threshold = threshold
        self.height_thresh = height_thresh

        self._hog = cv2.HOGDescriptor()
        self._people_detector = cv2.HOGDescriptor_getDefaultPeopleDetector()

        self._hog.setSVMDetector(self._people_detector)
 
    # Malisiewicz et al.
    # https://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
    # adapted to suppress by HOG weights instead of geometry
    # and use CV Rect (x, y, w, h) inputs
    def non_max_suppression_fast(self, boxes, weights, overlap_thresh):
        # if there are no boxes, return an empty list
        if len(boxes) == 0:
            return []

        # if the bounding boxes integers, convert them to floats --
        # this is important since we'll be doing a bunch of divisions
        if boxes.dtype.kind == "i":
            boxes = boxes.astype("float")

        # initialize the list of picked indexes	
        pick = []

        # grab the coordinates of the bounding boxes
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 0] + boxes[:, 2]
        y2 = boxes[:, 1] + boxes[:, 3]

        # compute the area of the bounding boxes and sort the bounding
        # boxes by the bottom-right y-coordinate of the bounding box
        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)

        # keep looping while some indexes still remain in the indexes
        # list
        while len(idxs) > 0:
            # grab the last index in the indexes list and add the
            # index value to the list of picked indexes
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)

            # find the largest (x, y) coordinates for the start of
            # the bounding box and the smallest (x, y) coordinates
            # for the end of the bounding box
            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])
            
            # compute the width and height of the bounding box
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            # compute the ratio of overlap
            overlap = (w * h) / area[idxs[:last]]

            # delete all indexes from the index list that have
            idxs = np.delete(idxs, np.concatenate(([last],
                    np.where(overlap > overlap_thresh)[0])))

        # return only the bounding boxes that were picked using the
        # integer data type
        return boxes[pick].astype("int")

    def detect(self, image):
        # https://www.pyimagesearch.com/2015/11/16/hog-detectmultiscale-parameters-explained/
        found, found_weights = self._hog.detectMultiScale(
                                    image,
                                    winStride=(8, 8),
                                    padding=(8, 8),
                                    scale=1.05
                               )

        found_nms = self.non_max_suppression_fast(found,
                            np.array(found_weights).flatten(),
                            overlap_thresh=0.4)

        detections = sorted(
                        filter(
                            lambda x: x[1] > self.threshold and x[0][3] > self.height_thresh,
                            zip(found_nms, found_weights)
                        ), key=lambda x: x[1]
                     )

        return detections

    def get_avg_depth_region(self, depth_image, region):
        return 0


class CvBridgeConverter:
    def __init__(self):
        self._bridge = CvBridge()

    def image_msg_to_cv(self, img_msg, fmt='bgr8'):
        try:
            return self._bridge.imgmsg_to_cv2(img_msg, fmt)
        except CvBridgeError as e:
            rospy.logerr(e)

        return None

    def cv_to_image_msg(self, cv_img, fmt='bgr8'):
        try:
            return self._bridge.cv2_to_imgmsg(cv_img, fmt)
        except CvBridgeError as e:
            rospy.logerr(e)

        return None


class ImageCallback:
    '''detects and publishes detections, used in callbacks'''
    def __init__(self, detector, det_pub, dist_pub, det_img_pub=None):
        self.detector = detector
        self.det_pub = det_pub
        self.dist_pub = dist_pub
        self.det_img_pub = det_img_pub
        self.cvbridge = CvBridgeConverter()
        self.last_roi = None

        self.num_images = 0
        self.num_detections = 0

    def __call__(self, msg, cb_type=0):
        # this is a slight hack to let us reuse this callback object for
        # multiple message types

        if cb_type == 0: # camera image -> check for person
            self.detect_roi(msg)
        elif cb_type == 1: # depth image -> get distance
            self.detect_depth(msg)

    def detect_roi(self, msg):
        image = self.cvbridge.image_msg_to_cv(msg)

        if image is not None:
            detections = self.detector.detect(image)

            self.num_images += 1

            if len(detections):
                (x, y, w, h), (weight,) = detections[0]

                # publish the "best" detection
                self.det_pub.publish(RegionOfInterest(x, y, h, w, False))
                self.last_roi = (x, y, w, h)

                # draw _all_ of the detection regions in the image
                if self.det_img_pub is not None:
                    for i, d in enumerate(detections):
                        (x, y, w, h), (weight,) = d

                        if i == 0:
                            color = (0, 255, 0)
                        else:
                            color = (0, 0, 255)
                        image = cv2.rectangle(image, (x, y), (x+w, y+h), color, 1)
                        image = cv2.putText(image, '{0}'.format(weight), (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                        image = cv2.putText(image, '{0}'.format(h), (x, y+h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # track for performance calculation
                self.num_detections += 1
            else:
                self.last_roi = None # no detections
            
            if self.det_img_pub is not None:
                self.det_img_pub.publish(self.cvbridge.cv_to_image_msg(image))

    def detect_depth(self, msg):
        image = self.cvbridge.image_msg_to_cv(msg, fmt='32FC1')

        if image is not None and self.last_roi is not None:
            x, y, w, h = self.last_roi

            # NOTE: heed my warning: OpenCV hath store thine images in column-major order
            # thusly, the y-axis be presented as the first indexes
            # know this and ye will save many an hour debugging
            depth_region = image[y:y+h, x:x+w]

            # FIXME: do the gradient thingy and find the peak at the middle row
            person_depth = depth_region[h/2, w/2]
            # person_depth = np.nanmax(depth_region) # alt: poor readings with walls

            distance = np.nanmean(
                            depth_region[
                                np.logical_and(
                                    depth_region < person_depth * 1.1,
                                    depth_region > person_depth * 0.8
                                )
                            ]
                       )
            self.dist_pub.publish(distance)


if __name__ == '__main__':
    rospy.init_node('pursuer_detector')
    ns = rospy.get_namespace()

    image_topic = rospy.get_param('~image', ns + 'camera/rgb/image_raw')
    depth_image_topic = rospy.get_param('~depth_image', ns + 'camera/depth/image_raw')
    detection_topic = rospy.get_param('~detections', '/pursuer_detections')
    distance_topic = rospy.get_param('~distance', '/pursuer_distance')
    det_image_topic = rospy.get_param('~detection_image', '/pursuer_detection_image')

    detector = PursuerDetector(threshold=0.5)

    detection_pub = rospy.Publisher(
        detection_topic,
        RegionOfInterest,
        queue_size=10
    )

    distance_pub = rospy.Publisher(
        distance_topic,
        Float32Msg,
        queue_size=10
    )

    det_image_pub = rospy.Publisher(
        det_image_topic,
        Image,
        queue_size=10
    )

    cb = ImageCallback(detector, detection_pub, distance_pub, det_image_pub)
    image_sub = rospy.Subscriber(image_topic, Image, cb, queue_size=1, callback_args=0)
    image_sub = rospy.Subscriber(depth_image_topic, Image, cb, queue_size=1, callback_args=1)
    
    # detection performance
    detperf_topic = rospy.get_param('~detetct_perf_topic', '/pursuer_detection_performance')
    detperf_pub = rospy.Publisher(
        detperf_topic,
        Float32Msg,
        queue_size=10
    )

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if cb.num_images > 0 and cb.num_detections > 0:
            print(cb.num_detections, cb.num_images)
            detperf_pub.publish(float(cb.num_detections) / cb.num_images)

        rate.sleep()        
            

