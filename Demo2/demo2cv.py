import smbus
import time
import board as board 
import numpy as np
import cv2 as cv
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA
offset = 0
#not_num = float("nan")
# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

bus = smbus.SMBus(1)
address = 0x05

def writeNumber(value):
   #bus.write_byte_data(address, offset, value)
    bus.write_byte(address, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 0)
    return number

#lcd.clear()
# Set LCD color to red
#lcd.color = [100, 0, 0]
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
writeNumber(100)
out_of_angle = 0
sent_counter = 0
count = 0
stage = 0
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    if stage>=2:
        low_blu = np.array([90,50,50])
        up_blu = np.array([110,255,255])
    else:
        low_blu = np.array([90,70,120])
        up_blu = np.array([110,110,140])
    #low_blu = np.array([90,50,50])
    #up_blu = np.array([110,255,255])
    mask = cv.inRange(hsv,low_blu,up_blu)
    img_msk = cv.bitwise_and(frame,frame,mask= mask)
    sp = (0,0)
    if stage==3:
        ep = (int(img_msk.shape[1]),int(img_msk.shape[0]*0.8))
    else:
        ep = (int(img_msk.shape[1]),int(img_msk.shape[0]*0.5))
    color = (0,0,0)
    thick = -1
    img_msk = cv.rectangle(img_msk,sp,ep,color,thick)
    #sp = (int(img_msk.shape[1]) - 50,0)
    #ep = (int(img_msk.shape[1]),int(img_msk.shape[0]))
    #img_msk = cv.rectangle(img_msk,sp,ep,color,thick)
    
    #img = (img_msk*1.0 / img_msk.mean(axis=(0,1)))
    img = img_msk
    
    #kern = np.ones((5,5),np.float32)/25
    #smooth = cv.filter2D(img_msk,-1,kern)
    #kern = np.ones((5,5),np.uint8)
    #smooth = cv.morphologyEx(cv.morphologyEx(img_msk, cv.MORPH_OPEN, kern), cv.MORPH_CLOSE, kern)
    #img = smooth
    
    gry = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(gry,50,255,cv.THRESH_BINARY)
    tup = np.nonzero(thresh)
    if not np.any(tup[0]) and stage==0:
        print('No markers found')
        if count == 0:
            #time.sleep(5)
            count = count + 1
        #lcd.message="Angle: None "
        #writeNumber(100)
        #lcd.clear
    #elif not out_of_angle:
    elif sent_counter<4:
        stage = 1
        x_zero = img.shape[1]*0.5
        y_zero = img.shape[0]*0.5
        x_coor = round(np.mean(tup[1]))
        y_coor = round(np.mean(tup[0]))
        angle = round(27*(x_zero-x_coor)/x_zero)
        #byteangle = 6
        #writeNumber(np.int8(-angle))
        if np.isnan(angle) or len(tup[1]) < 5:
            continue
        print('Angle: {:.2f}'.format(np.int8(-angle)))
        writeNumber(np.int8(-angle))
        #out_of_angle = readNumber()
        sent_counter = sent_counter + 1
        #if angle < 0:
            #lcd.message="Angle:{:.2f}   ".format(angle)
        #else:
            #lcd.message="Angle: {:.2f}  ".format(angle)
        #lcd.clear
    elif not out_of_angle:
        out_of_angle = readNumber()
        print(out_of_angle)
        stage = 2
        #print('Not here yet')
        #print(tup[0])
    elif img.shape[0]-1 in tup[0]:
        print('Here here')
        stage = 3
        #writeNumber(100)
    elif not np.any(tup[0]):
        print('Here')
        count = count + 1
        if count < 2:
            #time.sleep(4)
            continue
        elif count > 10:
            writeNumber(100)

    # Display the resulting frame
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        #lcd.clear
        break
    
 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()