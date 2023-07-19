import cv2
import numpy as np
# 1mm = 2.035 px
# slider length = 31.7 mm

# find the percent of movement for the desired length 
def convert_length_to_percent(slider_length, desired_length):
    return (desired_length * 100)/slider_length

def detect_triangle(frame, flag, color):
    # TODO: detect both triangles
    while not flag:
        into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        if into_hsv.any():
            flag = True

    # uncomment when getting back to the robot camera
    # L_limit=np.array([0, 0, 255]) # setting the yellow lower limit
    # U_limit=np.array([100, 50, 255]) # setting the yellow upper limit

    if color == 'yellow':

        L_limit=np.array([20, 0, 100]) # setting the yellow lower and upper limit
        U_limit=np.array([40, 20, 255])   
    
    if color == 'green':
        L_limit=np.array([50, 100, 100]) # setting the green lower and upper limit
        U_limit=np.array([70, 200, 255]) 

    b_mask=cv2.inRange(into_hsv,L_limit,U_limit)
    yellow=cv2.bitwise_and(frame,frame,mask=b_mask)

        
    contours = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    print(contours)
    # find the biggest contour 
    c = max(contours, key = cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c)

    return contours, yellow, x, y, radius

def contour_detect_blue_screen(frame):

    into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


    L_limit=np.array([80,50,50]) # setting the blue lower limit
    U_limit=np.array([139,255,255]) # setting the blue upper limit
            
    
    b_mask=cv2.inRange(into_hsv,L_limit,U_limit)
    blue=cv2.bitwise_and(frame,frame,mask=b_mask)

        
    contours = cv2.findContours(b_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # find the biggest contour 
    c = max(contours, key = cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c)

    return contours, blue, x, y, radius

# NOTE: debug functions
def webCam():
    cap = cv2.VideoCapture(0)
    
    while 1:
        ret,frame =cap.read() 
        contours, yellow, x1, y1, radius = detect_triangle(frame)

        output = cv2.drawContours(yellow, contours, -1, (0, 0, 255), 3)

        # output images
        cv2.imshow('Original',output) # to display the original frame
        cv2.imshow('yellow Detector',frame) # to display the yellow object output
        
        # exit from while 
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    
    cv2.destroyAllWindows()


def get_yellow_perc():

    cap = cv2.VideoCapture(0)
    _, image = cap.read()
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # for the display
    w = 0
    h = 0

    flag = False 

    # getting the yellow triangle
    contours, yellow, x1, y1, radius1 = detect_triangle(image, flag, 'yellow')
    black_strip_contour, blue, x, y, radius = contour_detect_blue_screen(image)

    for cnt in black_strip_contour:
        area = cv2.contourArea(cnt)
        if area > 55000 and area < 250000:
            cv2.drawContours(image, [cnt], -1, (0, 0, 255), 1)
            cv2.circle(image, (int(x), int(y)), 3, (0, 255, 255), 1)
            print("pronasao sam displ")
            cv2.imshow('display',image) # to display the original frame

            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            # if len(approx) == 4:
            _, _, w, h = cv2.boundingRect(cnt)    

    slider_length = h - 9.5
    percent_vals = []
     #ako nema zelene, racuna samo za zutu
    print("Stampam zuta")
    desired_length_yellow = y + h/2 - y1
    percent_output = convert_length_to_percent(slider_length, desired_length_yellow)
    percent_vals.append(percent_output)
    print(percent_output)

    return percent_output


def get_green_perc():

    cap = cv2.VideoCapture(0)
    _, image = cap.read()
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # for the display
    w = 0
    h = 0

    flag = False 

    # getting the yellow triangle
    contours, green, x1, y1, radius1 = detect_triangle(image, flag, 'green')
    black_strip_contour, blue, x, y, radius = contour_detect_blue_screen(image)

    for cnt in black_strip_contour:
        area = cv2.contourArea(cnt)
        if area > 55000 and area < 250000:
            cv2.drawContours(image, [cnt], -1, (0, 0, 255), 1)
            cv2.circle(image, (int(x), int(y)), 3, (0, 255, 255), 1)
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            # if len(approx) == 4:
            _, _, w, h = cv2.boundingRect(cnt)    

    slider_length = h - 9.5
    percent_vals = []
     #ako nema zelene, racuna samo za zutu
    print("Stampam zuta")
    desired_length_green = y + h/2 - y1
    percent_output = convert_length_to_percent(slider_length, desired_length_green)
    percent_vals.append(percent_output)
    print(percent_output)

    return percent_output   


def staticImage():

    cap = cv2.VideoCapture(0)
    _, image = cap.read()
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # for the display
    w = 0
    h = 0

    # load images
    # NOTE: vratiti na staro posle
    # image = _image

    flag = False 

    # getting the yellow triangle
    contours, yellow, x1, y1, radius1 = detect_triangle(image, flag, 'yellow')
    # getting the green triangle
    contours1, green, x2, y2, radius2 = detect_triangle(image, flag, 'green')

    # drawing yellow triangle contour
    output = cv2.drawContours(yellow, contours, -1, (0, 0, 255), 3)
    # cv2.imshow('Original',output) # to display the original frame

    # output = cv2.drawContours(green, contours1, -1, (0, 0, 255), 3)
    # cv2.imshow('green',output) # to display the original frame

    # verification on the original image
    image = cv2.circle(image, (int(x1), int(y1)), int(radius1), (0, 0, 0) , 3)
    image = cv2.circle(image, (int(x2), int(y2)), int(radius2), (255, 0, 0) , 3)

    black_strip_contour, blue, x, y, radius = contour_detect_blue_screen(image)

    # adding offset - definisano na osnovu slike gde se otprilike nalazi vrh
    # extracting the display based on the area formed by the contour
    for cnt in black_strip_contour:
        area = cv2.contourArea(cnt)
        if area > 55000 and area < 250000:
            cv2.drawContours(image, [cnt], -1, (0, 0, 255), 1)
            cv2.circle(image, (int(x), int(y)), 3, (0, 255, 255), 1)
            print("pronasao sam displ")
            cv2.imshow('display',image) # to display the original frame

            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            # if len(approx) == 4:
            _, _, w, h = cv2.boundingRect(cnt)


    # NOTE: promeniti logiku malo
    # getting the desired length and the length of the slider from the screen

    slider_length = h
    percent_vals = []
     #ako nema zelene, racuna samo za zutu
    print("Stampam zuta")
    desired_length_yellow = y + h/2 - y1
    percent_output = convert_length_to_percent(slider_length, desired_length_yellow)
    percent_vals.append(percent_output)
    print(int(percent_output))


    # ukoliko je detektovana zelena, gledaj samo krajnju vrednost
    print("stampam zelena")
    desired_length_green = y + h/2 - y2
    percent_output = convert_length_to_percent(slider_length, desired_length_green)
    percent_vals.append(percent_output)
    print(int(percent_output))

    
    # can be commented while being integrated
    # debug purpose only
    # cv2.imshow('Original',output) # to display the original frame
    # cv2.imshow('yellow and green Detector',image) # yellow triangle   
    # cv2.imshow('black strip Detector',image)    

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return percent_vals

# main process
def main():
    # NOTE: switch between these two depending on testing
    # webCam()
    staticImage()

if __name__ == "__main__":
    main()