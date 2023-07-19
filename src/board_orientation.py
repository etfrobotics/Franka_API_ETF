# this script should detect the board orientation and position within a picture
import cv2
import numpy as np
import time
# contour detection - circles

dist = np.array([0.2610, 0.4163, 0, 0, 0])
mtx = np.array([[2595.79350037846,	0,	650.746133050557],
[0,	2596.04867348007,	531.462343257144],
[0,	0,	1]])


def contour_detect_blue(frame):

    into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


    L_limit=np.array([98,50,50]) # setting the blue lower limit
    U_limit=np.array([139,255,255]) # setting the blue upper limit
            
    
    b_mask=cv2.inRange(into_hsv,L_limit,U_limit)
    blue=cv2.bitwise_and(frame,frame,mask=b_mask)

        
    contours = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # find the biggest contour 
    c = max(contours, key = cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c)

    return contours, blue, x, y, radius

def contour_detect_red(frame):

    into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


    L_limit=np.array([0, 30, 30]) # setting the blue lower limit
    U_limit=np.array([20, 255, 255]) # setting the blue upper limit
            
    
    b_mask=cv2.inRange(into_hsv,L_limit,U_limit)
    blue=cv2.bitwise_and(frame,frame,mask=b_mask)

        
    contours = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # find the biggest contour 
    c = max(contours, key = cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c)

    return contours, blue, x, y

# (X1, Y1) JE PLAVO, (X2, Y2) JE CRVENO
def orientation(x1, y1, x2, y2):
    # try:
        return np.arctan2((y2 - y1), (x2 - x1)) - np.pi/2
    # except:
    #     pass

def vision():
    x_calibration = 0.317908266562
    y_calibration = -0.229880543756

    # x_calibration = 0.263923879683
    # y_calibration = -0.280765012648

    cap = cv2.VideoCapture(0)
    cap.set(3,1280)
    cap.set(4,960)
    
    
    ret,frame =cap.read()
    # cv2.imshow('Original', frame)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (1280,960), 1, (1280,960)) 
    # frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    # cv2.imshow('Und', frame2-frame) 
    # contours, blue, x2, y2, radius2 = contour_detect_blue(frame)
    contours, blue, x1, y1, radius = contour_detect_blue(frame)
    # print(radius1)
    # print(x1, y1)
    # print(x2, y2)
    contours_r, red, x2, y2 = contour_detect_red(frame)

    print("plavo dugme")
    print(x1,y1)

    board_orientation = orientation(x1, y1, x2, y2)
    print("aruco prep")
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict,
        parameters=arucoParams)

    print("cosak") 
    # print(corners)

    output = cv2.drawContours(blue, contours, -1, (0, 0, 255), 3)
    # cv2.imshow('Original',output) # to display the original frame
    output2 = cv2.drawContours(red, contours_r, -1, (0, 0, 255), 3)

    # this will give the color to mask.
    cv2.imshow('Original',output) # to display the original frame
    # cv2.imshow('Original',output2) # to display the original frame

    cv2.imshow('Blue Detector',frame) # to display the blue object output

    if cv2.waitKey(1) == ord('q'):

        cap.release()
    
    norm_const_x = 2.84
    norm_const_y = 2.3
    # cv2.destroyAllWindows()    
    x1_coords = ((y1-corners[0][0][3][0])/1000) / (norm_const_x) + x_calibration
    y1_coords = ((x1-corners[0][0][3][1])/1000) / (norm_const_y) + y_calibration
    board_origin = [x1_coords, y1_coords]

    # board_origin = [x1_coords - 0.018475488 * np.cos(board_orientation) - 0.102318134 * np.sin(board_orientation), 
    # y1_coords + 0.018475488 * np.sin(board_orientation) - 0.102318134 * np.cos(board_orientation)]
    # board_origin = [x1_coords - 0.02141 * np.cos(board_orientation) - 0.09879 * np.sin(board_orientation), 
    # y1_coords + 0.02141 * np.sin(board_orientation) - 0.09879 * np.cos(board_orientation)]

    # time.sleep(20)

    
    return board_origin, board_orientation, frame


if __name__ == '__main__':
    pass 
    # main()