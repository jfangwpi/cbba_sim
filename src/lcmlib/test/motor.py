# import RPi.GPIO as GPIO
# import time

path_file = open("../../../build/bin/path_1.txt","r")


path = path_file.readlines()

for position in path:
	parsed_position = position.split(" ")
	ID = int(parsed_position[0])
	physical_coordinate = [float(parsed_position[0]), float(parsed_position[1])]
	print ID 
	print("\n")
	print physical_coordinate	
	print("\n")
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
# GPIO.setup(20,GPIO.OUT)
# GPIO.setup(21,GPIO.OUT)
# GPIO.setup(6,GPIO.OUT)
# GPIO.setup(13,GPIO.OUT)
# D1=12
# D2=26
# print("Please input speed pulse")
# PWM=input()
# GPIO.setup(D1,GPIO.OUT)
# GPIO.setup(D2,GPIO.OUT)
# p1=GPIO.PWM(D1,500)
# p2=GPIO.PWM(D2,500)
# p1.start(PWM)
# p2.start(PWM)
	


# def forward(tf):
# 	GPIO.output(20,True)
# 	GPIO.output(21,False)
# 	GPIO.output(13,True)
# 	GPIO.output(6,False)
# 	time.sleep(tf)
# 	GPIO.cleanup()


# def backward(tf):
# 	GPIO.output(20,False)
# 	GPIO.output(21,True)
# 	GPIO.output(13,False)
# 	GPIO.output(6,True)
# 	time.sleep(tf)
# 	GPIO.cleanup()
	
# def turnleft(tf):
# 	GPIO.output(20,False)
# 	GPIO.output(21,False)
# 	GPIO.output(6,False)
# 	GPIO.output(13,True)
# 	time.sleep(tf)
# 	GPIO.cleanup()
	
# def turnright(tf):
# 	GPIO.output(20,True)
# 	GPIO.output(21,False)
# 	GPIO.output(6,False)
# 	GPIO.output(13,False)
# 	time.sleep(tf)
# 	GPIO.cleanup()
	
# print("please input your direction: 1 forward; 2 backward; 3 right; 4 left")	
# key=input()
# if key == 1:
# 	print("forward")
# 	forward(10)
# elif key == 2:
# 	print("backward")
# 	backward(10)
# elif key == 3:
# 	print("turnright")
# 	turnright(10)
# elif key == 4:
# 	print("turnleft")
# 	turnleft(10)
