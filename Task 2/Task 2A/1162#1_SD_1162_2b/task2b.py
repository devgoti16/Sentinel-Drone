import numpy as np
import cv2
lower = np.array([15,150,20])
upper = np.array([45,255,255])
img = cv2.imread('yellow_detect.jpeg')
image = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
mask = cv2.inRange(image , lower , upper)
contours,hierarchy = cv2.findContours(mask ,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
if len(contours) != 0:
    for i in contours:
        if cv2.contourArea(i) > 500:
            x,y,w,h = cv2.boundingRect(i)
            M= cv2.moments(i)
            cx = -1
            cy = -1
            if (M['m00'] != 0):
            	cx = int(M['m10']/M['m00'])
            	cy = int(M['m01']/M['m00'])
print(int(x+(w/2)),int(y+(h/2))) 
print(int(cx) , int(cy))
cv2.imshow("mask",mask)
#cv2.imshow("mask",mask)        
cv2.waitKey(0)
cv2.destroyAllWindows
