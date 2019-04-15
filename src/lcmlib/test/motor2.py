import RPi.GPIO as GPIO
import time

#print("Please input speed pulse")
#PWM=input()
PWM=25
D1=12
D2=26
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(D1,GPIO.OUT)
GPIO.setup(D2,GPIO.OUT)
p1=GPIO.PWM(D1,500)
p2=GPIO.PWM(D2,500)
p1.start(PWM)
p2.start(PWM)

def backward(tf):
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(20,GPIO.OUT)
	GPIO.setup(21,GPIO.OUT)
	GPIO.setup(6,GPIO.OUT)
	GPIO.setup(13,GPIO.OUT)
	GPIO.setup(D1,GPIO.OUT)
	GPIO.setup(D2,GPIO.OUT)
	p1.start(PWM)
	p2.start(PWM)
	GPIO.output(20,False)
	GPIO.output(21,True)
	GPIO.output(13,False)
	GPIO.output(6,True)
	time.sleep(tf)
	GPIO.cleanup()

def forward(tf):
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(20,GPIO.OUT)
	GPIO.setup(21,GPIO.OUT)
	GPIO.setup(6,GPIO.OUT)
	GPIO.setup(13,GPIO.OUT)
	GPIO.setup(D1,GPIO.OUT)
	GPIO.setup(D2,GPIO.OUT)
	p1.start(PWM)
	p2.start(PWM)
	GPIO.output(20,True)
	GPIO.output(21,False)
	GPIO.output(13,True)
	GPIO.output(6,False)
	time.sleep(tf)
	GPIO.cleanup()
	
def turnleft(tf):
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(20,GPIO.OUT)
	GPIO.setup(21,GPIO.OUT)
	GPIO.setup(6,GPIO.OUT)
	GPIO.setup(13,GPIO.OUT)
	GPIO.setup(D1,GPIO.OUT)
	GPIO.setup(D2,GPIO.OUT)
	p1.start(PWM)
	p2.start(PWM)
	GPIO.output(20,False)
	GPIO.output(21,False)
	GPIO.output(6,False)
	GPIO.output(13,True)
	time.sleep(tf)
	GPIO.cleanup()
	
def turnright(tf):
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(20,GPIO.OUT)
	GPIO.setup(21,GPIO.OUT)
	GPIO.setup(6,GPIO.OUT)
	GPIO.setup(13,GPIO.OUT)
	GPIO.setup(D1,GPIO.OUT)
	GPIO.setup(D2,GPIO.OUT)
	p1.start(PWM)
	p2.start(PWM)
	GPIO.output(20,True)
	GPIO.output(21,False)
	GPIO.output(6,False)
	GPIO.output(13,False)
	time.sleep(tf)
	GPIO.cleanup()

#-------------- test speed and calculate time for going through one cell---------------#
#------ fixed PWM, test the distance in 5 seconds for 5 times----#
'''
dis1=
dis2=
dis3=
dis4=
dis5=
speed=(dis1+dis2+dis3+dis4+dis5)/5/5
cell_width=
Tt=cell_width/speed # time for going straight
'''
path_file = open("../../../build/bin/path_1.txt","r")


path = path_file.readlines()
i = 0
path_id = [0] * len(path_id)
path_physical_coordinate = [0] * len(path)

for position in path:
	parsed_position = position.split(" ")
	path_id[i] = int(parsed_position[0])
	path_physical_coordinate[i] = [float(parsed_position[1]), float(parsed_position[2])]
	i = i + 1

print("\n")
print path_id 
print("\n")
print path_physical_coordinate	

dimension=5
#path_id=[20,21,22,23,18,13,8,7,6,5,0]

i=1
print("No.1 step is forward")
#forward(Tt)# command for 1st(start) cell
while i <= len(path_id):
	if abs(path_id[i]-path_id[i-1]) == abs(path_id[i]-path_id[i+1]):
		PWM=25
		a=i+1
		print('No.%d step is forward'%a)
		forward(Tt)
	elif path_id[i-1]-path_id[i] == dimension and path_id[i+1]-path_id[i] == 1 :
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_right'%a)
		turnright(1.8)
	elif path_id[i-1]-path_id[i] == -1 and path_id[i+1]-path_id[i] == dimension :
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_right'%a)
		turnright(1.8)
	elif path_id[i-1]-path_id[i] == -dimension and path_id[i+1]-path_id[i] == -1 :
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_right'%a)
		turnright(1.8)
	elif path_id[i-1]-path_id[i] == 1 and path_id[i+1]-path_id[i] == -dimension :
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_right'%a)
		turnright(1.8)
	elif path_id[i-1]-path_id[i] == dimension and path_id[i+1]-path_id[i] == -1:
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_left'%a)
		turnleft(1.8)
	elif path_id[i-1]-path_id[i] == -1 and path_id[i+1]-path_id[i] == -dimension:
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_left'%a)
		turnleft(1.8)
	elif path_id[i-1]-path_id[i] == -dimension and path_id[i+1]-path_id[i] == 1:
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_left'%a)
		turnleft(1.8)
	elif path_id[i-1]-path_id[i] == 1 and path_id[i+1]-path_id[i] == dimension:
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(20,GPIO.OUT)
		GPIO.setup(21,GPIO.OUT)
		GPIO.setup(6,GPIO.OUT)
		GPIO.setup(13,GPIO.OUT)
		GPIO.setup(D1,GPIO.OUT)
		GPIO.setup(D2,GPIO.OUT)
		PWM=50
		p1.start(PWM)
		p2.start(PWM)
		GPIO.cleanup()
		a=i+1
		print('No.%d step is turn_left'%a)
		turnleft(1.8)
	i=i+1  
