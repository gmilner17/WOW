#####################################
# Group 11
# EENG 350
# Spring 2022
# Dr. Sager
# Mini Project
# This code sends the setpoint to the Ardunio
#####################################

import smbus
import time
import numpy as np
import cv2 as cv
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
# Set LCD color to red
lcd.color = [100, 0, 0]

quad_lcd = ['3pi/2', ' pi  ', ' pi/2', '2pi  ']
old_quad = -1

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

bus = smbus.SMBus(1)
address = 0x05

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

while True:
    #Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    low_yel = np.array([20,100,100])
    up_yel = np.array([30,255,255])
    mask = cv.inRange(hsv,low_yel,up_yel)
    img_msk = cv.bitwise_and(frame,frame,mask= mask)
    
    
    img = img_msk
    
    gry = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(gry,50,255,cv.THRESH_BINARY)
    tup = np.nonzero(thresh)
    quad = 0
    if not np.any(tup[0]):
        print('No markers found')
        quad = -1
    else:
        x_zero = img.shape[1]*0.5
        y_zero = img.shape[0]*0.5
        x_coor = round(np.mean(tup[1]))
        y_coor = round(np.mean(tup[0]))
        if x_zero < x_coor and y_zero >= y_coor:
            quad = 1
        elif x_zero >= x_coor and y_zero > y_coor:
            quad = 2
        elif x_zero > x_coor and y_zero <= y_coor:
            quad = 3
        elif x_zero <= x_coor and y_zero < y_coor:
            quad = 4
        
    
    if old_quad != quad:     
        lcd.message="Set Point: %s"%(quad_lcd[quad - 1])
        old_quad = quad
    
    # Display the resulting frame
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break
    
    var = quad
    if not var:
        continue
    
    writeNumber(var)
     
    
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
