from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import imutils
import dht11
import datetime
import socket
 
s = socket.socket()         # Soket oluşturuldu
s.connect(('192.168.43.157', 12346))
g=socket.socket()
g.connect(('192.168.43.157', 12345))
i=socket.socket()
i.connect(('192.168.43.157', 12347))
time.sleep(1)
defaultSpeed = 18	
windowCenter = 320
centerBuffer = 10
pwmBound = float(50)
cameraBound = float(320)
kp = pwmBound / cameraBound
leftBound = int(windowCenter - centerBuffer)
rightBound = int(windowCenter + centerBuffer)
error = 0
ballPixel = 0
 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
 
#Pin tanımlama
rightFwd = 13
rightRev = 15
leftFwd = 33
leftRev = 37
GPIO_TRIGGER = 16
GPIO_ECHO = 18
instance = dht11.DHT11(pin=8)
 
#GPIO tanımlanması
GPIO.setup(leftFwd, GPIO.OUT)
GPIO.setup(leftRev, GPIO.OUT)
GPIO.setup(rightFwd, GPIO.OUT)
GPIO.setup(rightRev, GPIO.OUT)
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
GPIO.setup(GPIO_ECHO,GPIO.IN)
GPIO.setup(40,GPIO.IN)
GPIO.setup(38,GPIO.IN)
GPIO.output(leftFwd, False)
GPIO.output(leftRev, False)
GPIO.output(rightFwd, False)
GPIO.output(rightRev, False)
 
#PWM tanımlama
 
rightMotorFwd = GPIO.PWM(rightFwd, 50)
leftMotorFwd = GPIO.PWM(leftFwd, 50)
rightMotorRev = GPIO.PWM(rightRev, 50)
leftMotorRev = GPIO.PWM(leftRev, 50)
rightMotorFwd.start(defaultSpeed)
leftMotorFwd.start(defaultSpeed)
leftMotorRev.start(defaultSpeed)
rightMotorRev.start(defaultSpeed)
def updatePwm(leftPwm, rightPwm):
	rightMotorFwd.ChangeDutyCycle(rightPwm)
	leftMotorFwd.ChangeDutyCycle(leftPwm)
 
def turnRight(saggeri, solileri):
    rightMotorRev.ChangeDutyCycle(saggeri)
    leftMotorFwd.ChangeDutyCycle(solileri)
def pwmStop():
	rightMotorFwd.ChangeDutyCycle(0)
	rightMotorRev.ChangeDutyCycle(0)
	leftMotorFwd.ChangeDutyCycle(0)
	leftMotorRev.ChangeDutyCycle(0)
def distance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
 
    return distance
        
#Kamera kurulumu
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size = (640, 480))
 
time.sleep(0.1)
 
lower_red = np.array([0, 70, 90])
upper_red = np.array([15, 255, 255])
 
lower_green = np.array([45, 75, 85])
upper_green = np.array([70, 255, 255])
 
lower_blue = np.array([95,50,50])
upper_blue = np.array([120,255,255])
 
updatePwm(50, 50)
time.sleep(.4)
pwmStop()
 
time.sleep(.5)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break
	gaz=GPIO.input(38)
	isik=GPIO.input(40)
	image = frame.array
	output = image.copy()
	output_green = image.copy()
	output_blue = image.copy()
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_red, upper_red)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	output = cv2.bitwise_and(output, output, mask=mask)
	
	#yesil renk takibi
    
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	mask_green = cv2.erode(mask_green, None, iterations=2)
	mask_green = cv2.dilate(mask_green, None, iterations=2)
	output_green = cv2.bitwise_and(output_green, output_green,   mask=mask_green)
                #mavi renk takibi
 
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	mask_blue = cv2.erode(mask_blue, None, iterations=2)
	mask_blue = cv2.dilate(mask_blue, None, iterations=2)
	output_blue = cv2.bitwise_and(output_blue, output_blue, mask=mask_blue)
	
	gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
	gray_blue = cv2.cvtColor(output_blue, cv2.COLOR_BGR2GRAY)
	gray_green = cv2.cvtColor(output_green, cv2.COLOR_BGR2GRAY)
	
#çember çizme fonksiyonu
 
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 500, minRadius = 10, maxRadius = 200, param1 = 100,  param2 = 60)
	circles_green = cv2.HoughCircles(gray_green, cv2.HOUGH_GRADIENT, 3, 500, minRadius = 5, maxRadius = 200, param1 = 100,  param2 = 60)
	circles_blue = cv2.HoughCircles(gray_blue, cv2.HOUGH_GRADIENT, 3, 500, minRadius = 10, maxRadius = 200, param1 = 100,  param2 = 60)
	ballPixel = 0
	
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, radius) in circles:
			cv2.circle(output, (x, y), radius, (0, 255, 0), 2)		
			if radius > 10:	
				ballPixel = x
			else:
				ballPixel = 0
	if circles_green is not None:
		circles_green = np.round(circles_green[0, :]).astype("int")
		for (z, t, radius) in circles_green:
			cv2.circle(output_green, (z,t), radius, (0, 0,255), 2)
		
			if radius > 5:	
				ballPixel = z
			else:
				ballPixel = 0
	
	if circles_blue is not None:
		circles_blue = np.round(circles_blue[0, :]).astype("int")
		for (z, t, radius) in circles_blue:
			cv2.circle(output_blue, (z,t), radius, (255, 0,0), 2)
		
			if radius > 5:	
				ballPixel = z
			else:
                            ballPixel = 0
 
	cv2.imshow("output", output)
	#cv2.imshow("window",image)
	cv2.imshow("output_green", output_green)
	cv2.imshow("output_blue", output_blue)
	#key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
	mesafe=distance()
	print("mesafe: ", mesafe)
	#Proportional controller
	#time.sleep(0.1)
	#print(output_green)
	if ballPixel == 0 and mesafe >30:
		print ("rengi bul!")
		updatePwm(defaultSpeed, defaultSpeed)
		#time.sleep(0.2)
		#pwmStop()
		error = 0
	elif ballPixel == 0 and mesafe <30 :
                		print ("engel var")
              		pwmStop()
               		time.sleep(1)
                		turnRight(70, 70)
                		time.sleep(0.5)
                		pwmStop()      
	elif (ballPixel < leftBound) or (ballPixel > rightBound):
		error = windowCenter - ballPixel
		pwmOut = abs(error * kp) 
		#print ballPixel
		turnPwm = pwmOut + defaultSpeed
		if  ballPixel < (leftBound):
                   		 print ("right side")
                   		 if radius > 50 and ballPixel < 110 :
                        			print ("ballPixel")
                        		updatePwm(defaultSpeed, 8)
 
                        if circles_green is not None and mesafe< 30:
                            pwmStop()
                            print("sicaklik olculuyor..")
                            time.sleep(1)
                            result = instance.read()
                            if result.is_valid():
                                print("Last valid input: " + str(datetime.datetime.now()))
                                print("Temperature: %d C" % result.temperature)
                                print("Humidity: %d %%" % result.humidity)
                                sicaklik=str(result.temperature)
		s.sendall(sicaklik.encode())
		nem=str(result.humidity)
		s.sendall(nem.encode())
                                #time.sleep(1)
                                turnRight(70, 70)
                                time.sleep(0.5)
                                pwmStop()
                        elif circles is not None and mesafe<30:
                            pwmStop()
                            print("gaz olculuyor..")
                            time.sleep(1)
			    if gaz==1:
				    print("gaz kacagi var")
				    gas= 'gaz kacagi var.'
				    g.sendall(gas.encode())
			    else:
				    print("gaz kacagi yok")
				    gas= 'gaz kacagi yok.'
				    g.sendall(gas.encode())
                            turnRight(70, 70)
                            time.sleep(0.5)
                            pwmStop()
			elif circles_blue is not None and mesafe<30:
				pwmStop()
				print("isik siddeti olculuyor")
				time.sleep(1)
				if isik==0:
					print("ortam aydinlik")
					light='ortam aydinlik'
					i.sendall(light.encode())
				else:
					print("ortam karanlik")
					light='ortam karanlik'
					i.sendall(light.encode())
				turnRight(70,70)
				time.sleep(0.5)
				pwmStop()		
			    
                    else:
                        updatePwm(turnPwm, defaultSpeed)
                        print("sola donuyor")
                        if circles_green is not None and mesafe<50:
                            pwmStop()
                            print("sicaklik olculuyor..")
			    #time.sleep(1)
                            result = instance.read()
                            if result.is_valid():
                                print("Last valid input: " + str(datetime.datetime.now()))
                                print("Temperature: %d C" % result.temperature)
                                print("Humidity: %d %%" % result.humidity)
				sicaklik=str(result.temperature)
				s.sendall(sicaklik.encode())
				nem=str(result.humidity)
				s.sendall(nem.encode())
                                #time.sleep(1)
                                turnRight(70, 70)
                                time.sleep(0.5)
                                pwmStop()
                        if circles is not None and mesafe<30:
                            pwmStop()
                            print("gaz olculuyor..")
			    if gaz==1:
				    print("gaz kacagi var")
				    gas= 'gaz kacagi var.'
				    g.sendall(gas.encode())
			    else:
				    print("gaz kacagi yok")
				    gas= 'gaz kacagi yok.'
				    g.sendall(gas.encode())
			    
                            time.sleep(1)
                            turnRight(70, 70)
                            time.sleep(0.5)
                            pwmStop()
			elif circles_blue is not None and mesafe<30:
				pwmStop()
				print("isik siddeti olculuyor")
				time.sleep(1)
				if isik==0:
					print("ortam aydinlik")
					light='ortam aydinlik'
					i.sendall(light.encode())
				else:
					print("ortam karanlik")
					light='ortam karanlik'
					i.sendall(light.encode())
				turnRight(70,70)
				time.sleep(0.5)
				pwmStop()	
			    
		elif ballPixel > (rightBound):
			print ("left side")
			if radius > 50 and ballPixel > 540:
				print ("ballPixel")			    
				updatePwm(8, defaultSpeed)
				if circles_green is not None and mesafe<50:
                                    pwmStop()
                                    print("sicaklik olculuyor")
				    result = instance.read()
				    if result.is_valid():
			    		    sicaklik=str(result.temperature)
					    s.sendall(sicaklik.encode())
					    nem=str(result.humidity)
					    s.sendall(nem.encode())
					    #time.sleep(1)
					    turnRight(70, 70)
					    time.sleep(0.5)
					    pwmStop()
                                elif circles is not None and mesafe<30:
                                    pwmStop()
                                    print("gaz olcuyor")
                                    time.sleep(1)
				    if gaz==1:
					    print("gaz kacagi var")
					    gas= 'gaz kacagi var.'
					    g.sendall(gas.encode())
				    else:
					    print("gaz kacagi yok")
					    gas= 'gaz kacagi yok.'
					    g.sendall(gas.encode())
                                    turnRight(70,70)
                                    time.sleep(.5)
                                    pwmStop()
				elif circles_blue is not None and mesafe<30:
					pwmStop()
					print("isik siddeti olculuyor")
					time.sleep(1)
					if isik==0:
						print("ortam aydinlik")
						light='ortam aydinlik'
						i.sendall(light.encode())
					else:
						print("ortam karanlik")
						light='ortam karanlik'
						i.sendall(light.encode())
					turnRight(70,70)
					time.sleep(0.5)
					pwmStop()	  
			else:
				updatePwm(defaultSpeed, turnPwm)
				print("saga donuyor.")
				if circles_green is not None and mesafe<50:
                                   		    pwmStop()
                                   		    print("sicaklik olculuyor")
				    result = instance.read()
				    if result.is_valid():
					    sicaklik=str(result.temperature)
					    s.sendall(sicaklik.encode())
					    nem=str(result.humidity)
					    s.sendall(nem.encode())
					    #time.sleep(1)
					    turnRight(70, 70)
					    time.sleep(0.5)
					    pwmStop()
                                elif circles is not None and mesafe<30:
					pwmStop()
					print("gaz olculuyor")
					time.sleep(1)
					if gaz==1:
						print("gaz kacagi var")
						gas= 'gaz kacagi var.'
						g.sendall(gas.encode())
					else:
						print("gaz kacagi yok")
						gas= 'gaz kacagi yok.'
						g.sendall(gas.encode())
					turnRight(70,70)
					time.sleep(.5)
					pwmStop()
				elif circles_blue is not None and mesafe<30:
					pwmStop()
					print("isik siddeti olculuyor")
					time.sleep(1)
					if isik==0:
						print("ortam aydinlik")
						light='ortam aydinlik'
						i.sendall(light.encode())
					else:
						print("ortam karanlik")
						light='ortam karanlik'
						i.sendall(light.encode())
					turnRight(70,70)
					time.sleep(0.5)
					pwmStop()	   
else:
            
            print ("middle")
            if (radius < 100):
                updatePwm(defaultSpeed, defaultSpeed)
                print("uzaktasin devam et!")
            if circles_green is not None and mesafe <60:
                pwmStop()
                time.sleep(1)
                result = instance.read()
                if result.is_valid():
                    print("Last valid input: " + str(datetime.datetime.now()))
                    print("Temperature: %d C" % result.temperature)
                    print("Humidity: %d %%" % result.humidity)
		    sicaklik=str(result.temperature)
		    s.sendall(sicaklik.encode())
		    nem=str(result.humidity)
		    s.sendall(nem.encode())
		    turnRight(70, 70)
		    time.sleep(0.5)
		    pwmStop()	    
            if circles is not None and mesafe<30:
		    print("gaz olculuyor")
		    pwmStop()
		    time.sleep(1)
		    if gaz==1:
			    print("gaz kacagi var")
			    gas= 'gaz kacagi var.'
			    g.sendall(gas.encode())
		    else:
			    print("gaz kacagi yok")
			    gas= 'gaz kacagi yok.'
			    g.sendall(gas.encode())
		    turnRight(70,70)
		    time.sleep(0.5)
		    pwmStop()
	    elif circles_blue is not None and mesafe<30:
		     pwmStop()
		     print("isik siddeti olculuyor")
		     time.sleep(1)
		     if isik==0:
			     print("ortam aydinlik")
			     light='ortam aydinlik'
			     i.sendall(light.encode())
		     else:
			     print("ortam karanlik")
			     light='ortam karanlik'
			     i.sendall(light.encode())
		     turnRight(70,70)
		     time.sleep(0.5)
		     pwmStop()	
                
cv2.destroyAllWindows()
camera.close()
pwmStop()
GPIO.cleanup()
