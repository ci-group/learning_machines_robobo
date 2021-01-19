import cv2
import numpy as np

def detect(image):
    mask = cv2.inRange(image, (0, 140, 0), (20, 255, 20))
    masked = cv2.bitwise_and(image, image, mask=mask)
    # cv2.imshow('masked', masked)
    #
    # cv2.waitKey(0)  # waits until a key is pressed
    # cv2.destroyAllWindows()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    M = cv2.moments(blob)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    fast = cv2.FastFeatureDetector_create()
    fast.setNonmaxSuppression(0)
    fast.setThreshold(20)
    kp = fast.detect(image, None)
    xs = []
    ys = []
    for i in cv2.KeyPoint_convert(kp):
        xs.append(i[0])
        ys.append(i[1])
    maxkeypointid = np.argmax(ys)  # the y value of the closest block is always higher then those for the others
    # return xs[maxkeypointid], ys[maxkeypointid]

    return center[0], center[1]