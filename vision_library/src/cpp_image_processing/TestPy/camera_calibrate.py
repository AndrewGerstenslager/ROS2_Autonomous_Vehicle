import cv2
import numpy as np
import os
#convert float tuples to int tuples
def convert_tup(x):
    y=((int)(x[0]),(int)(x[1]))
    return y

#highlight points, debug purposes
def highlight_points(im,top_left,top_right,bottom_left,bottom_right):
    img=im.copy()
    cv2.circle(img, convert_tup(top_left), radius=5, color=(0, 0, 255), thickness=-1)
    cv2.circle(img, convert_tup(top_right), radius=5, color=(0, 255, 0), thickness=-1)
    cv2.circle(img, convert_tup(bottom_left), radius=5, color=(255, 0, 0), thickness=-1)
    cv2.circle(img, convert_tup(bottom_right), radius=5, color=(120, 0, 120), thickness=-1)
    return img

#Check if the checkerboard through the camera is parallel to the camera
def check_horizontal_checkerboard(top_left,top_right,bottom_right,bottom_left,threshold=5):
    global checkerboard_shape
    if (top_left[1]<bottom_right[1]) and (top_left[0]<bottom_right[0]) and (abs(top_left[1]-top_right[1])<=threshold):
        return True
    if (top_left[1]<bottom_right[1]) and (top_left[0]>bottom_right[0]) and (abs(top_left[1]-bottom_left[1])<threshold):
        checkerboard_shape=(checkerboard_shape[1],checkerboard_shape[0])
    return False

def print_at(x, y, string):
    print("\033["+str(y)+";"+str(x)+"H"+string)
    
video = cv2.VideoCapture(0)
os.system("cls")
#im=cv2.imread("test3.png")
checkerboard_horizontal=False
size_per_square = 1
checkerboard_shape = (4,3)
while not checkerboard_horizontal:
    #os.system("cls")
    status, im = video.read()
    width=checkerboard_shape[0]
    height=checkerboard_shape[1]
    im_copy=im.copy()
    ret, corners = cv2.findChessboardCorners(im, checkerboard_shape, None)

    print_at(0,1,"not found")
    if ret:
        top_left = tuple(corners[0][0])
        top_right = tuple(corners[width-1][0])
        bottom_left = tuple(corners[width*(height-1)][0])
        bottom_right = tuple(corners[width*height-1][0])
        
        checkerboard_horizontal = check_horizontal_checkerboard(top_left,top_right,bottom_right,bottom_left)
        print_at(0,2,"found checkerboard")
        
        if checkerboard_horizontal:
            print_at(0,3,"checkerboard is parallel to camera")
        points = np.array([top_left, bottom_left, bottom_right,top_right], dtype=np.float32)
        y_mid = im.shape[0] / 2
        x_mid = im.shape[1] / 2
        size=100
        ipm_pts = np.array([[x_mid - size, y_mid - size],[x_mid - size, y_mid + size],[x_mid + size, y_mid + size],[x_mid + size, y_mid - size]], dtype=np.float32)
        ipm_matrix = cv2.getPerspectiveTransform(points, ipm_pts)
        ipm = cv2.warpPerspective(im, ipm_matrix, im.shape[:2][::-1])

        im_copy=highlight_points(im,top_left,top_right,bottom_left,bottom_right)
    else:
        ipm=im
    cv2.imwrite("frame.png",im_copy)
    cv2.imwrite("result2.png", ipm)
    cv2.imshow("bruhin",im_copy)
    cv2.imshow("bruhout",ipm)
    cv2.waitKey(1)
    
print(ipm_matrix)

    
