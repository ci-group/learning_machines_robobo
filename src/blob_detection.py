import cv2
import numpy as np
import matplotlib.pyplot as plt
def determine_closeness_blob(contour):
    score = 0
    for i in contour:
        if i[0][1] >score:
            score = i[0][1]
    return score




def detect(image):
    mask = cv2.inRange(image, (0, 140, 0), (20, 255, 20))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        blob = max(contours, key=determine_closeness_blob)
        blob = np.array(blob)
        center = np.mean(blob.squeeze(axis=1), axis=0)
        # M = cv2.moments(blob)
        # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    else: 
        return False,False

    return center[0], center[1]


if __name__ == '__main__':
    image = cv2.imread('test')