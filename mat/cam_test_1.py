import dlib
import cv2
import numpy as np
import time
import sys
import threading
import RPi.GPIO as GPIO
from time import sleep


global graph
graph = 0
arr_x = np.arange(0,1,0.01)
arr_y = arr_x * 0
"""
f = open('graph.txt', 'r')
for i in range(100):
	arr_y[i] = int(f.readline())
f.close()"""
f = open('graph.txt', 'w')
#motor
servoPin1 = 13 #와이퍼
servoPin2 = 16 #L고정대
servoPin3 = 15 #R고정대
servoPin4 = 11 #바퀴 돌아가는 부분
servoPin5 = 12 #바퀴를 움직이는 목부분

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin1, GPIO.OUT)
GPIO.setup(servoPin2, GPIO.OUT)
GPIO.setup(servoPin3, GPIO.OUT)
GPIO.setup(servoPin4, GPIO.OUT)
GPIO.setup(servoPin5, GPIO.OUT)

servo1 = GPIO.PWM(servoPin1, 50)
servo2 = GPIO.PWM(servoPin2, 50)
servo3 = GPIO.PWM(servoPin3, 50)
servo4 = GPIO.PWM(servoPin4, 50)
servo5 = GPIO.PWM(servoPin5, 50)

servo1.start(0)
servo2.start(0)
servo3.start(0)
servo4.start(0)
servo5.start(0)

def stop_motor(t):
	servo1.ChangeDutyCycle(0)
	servo2.ChangeDutyCycle(0)
	servo3.ChangeDutyCycle(0)
	servo4.ChangeDutyCycle(0)
	servo5.ChangeDutyCycle(0)
	sleep(t)

current_state = 'r'
middle = 7.53
l_speed = middle+0.3
r_speed = middle-0.3

w_middle = 7.56
wl_speed = w_middle+0.35
wr_speed = w_middle-0.35

fr_speed = 7.38
fl_speed = 6.9

def next_page():
	global current_state
	#1
	servo4.ChangeDutyCycle(0)
	servo5.ChangeDutyCycle(r_speed)
	if current_state == 'l':
		servo1.ChangeDutyCycle(wr_speed) #와이퍼 R로 이동
		sleep(0.6) #바퀴 L-> R로 내려감
		servo5.ChangeDutyCycle(0)
		sleep(0.3)
	else:
		sleep(0.3) #바퀴 R -> R로 내려감

	stop_motor(1)

	#2
	servo4.ChangeDutyCycle(6.8) #바퀴 시계방향으로 돌음
	servo5.ChangeDutyCycle(0)
	sleep(0.4)

	stop_motor(0.5)

	#3
	servo2.ChangeDutyCycle(fl_speed)  # 왼쪽 고정대 올라감
	sleep(0.5)

	stop_motor(0.5)  # 잠깐 정지

	#4
	servo1.ChangeDutyCycle(wl_speed)  # 와이퍼가 책페이지로 들어감
	sleep(0.4)
        
	servo5.ChangeDutyCycle(l_speed)
	servo4.ChangeDutyCycle(0) #바퀴 살짝 들어줌(와이퍼랑 동시에)
	sleep(0.2)
        
	servo5.ChangeDutyCycle(0) 
	sleep(0.4)

	stop_motor(1)  # 잠깐 정지

	#5
	servo2.ChangeDutyCycle(fr_speed)  # 왼쪽 고정대 내려감
	sleep(0.5)

	stop_motor(1)  # 잠깐 정지

	servo1.ChangeDutyCycle(wr_speed)  # 와이퍼 오른쪽으로 원위치
	sleep(1)
	
	servo1.ChangeDutyCycle(0)
	servo2.ChangeDutyCycle(0)
	servo3.ChangeDutyCycle(0)
	servo4.ChangeDutyCycle(0)
	servo5.ChangeDutyCycle(0)
	current_state = 'r'

def prev_page():
	global current_state
	#1
	servo4.ChangeDutyCycle(0)
	servo5.ChangeDutyCycle(l_speed)
	if current_state == 'l':
		sleep(0.3) #바퀴 L->L로 내려감
	else:
		servo1.ChangeDutyCycle(wl_speed)  # after wheel goes left
		sleep(0.6)
		servo5.ChangeDutyCycle(0)
		sleep(0.3) #바퀴 R->L로 내려감

	stop_motor(1)

	#2
	servo4.ChangeDutyCycle(8) #바퀴 반시계방향으로 굴러줌
	servo5.ChangeDutyCycle(0)
	sleep(0.4)

	stop_motor(0.5)

	#3
	servo3.ChangeDutyCycle(7.5)  # 오른쪽 고정대 올라감
	sleep(0.5)

	stop_motor(0.5)  # 잠깐 정지

	#4
	servo1.ChangeDutyCycle(wr_speed)  # 와이퍼가 책페이지로 들어감
	sleep(0.4)
        
	servo5.ChangeDutyCycle(r_speed)
	servo4.ChangeDutyCycle(0) #바퀴 살짝 들어줌(와이퍼랑 동시에)
	sleep(0.2)
        
	servo5.ChangeDutyCycle(0) #와이퍼가 책을 넘김
	sleep(0.4)

	stop_motor(1)

	#5
	servo3.ChangeDutyCycle(6.5)  # 오른쪽 고정대 내려감
	sleep(0.5)

	stop_motor(1)  # 잠깐 정지

	servo1.ChangeDutyCycle(wl_speed);  # 와이퍼 왼쪽으로 원위치
	sleep(1)

	servo1.ChangeDutyCycle(0)
	servo2.ChangeDutyCycle(0)
	servo3.ChangeDutyCycle(0)
	servo4.ChangeDutyCycle(0)
	servo5.ChangeDutyCycle(0)
	current_state = 'l'

# 두 점 사이의 거리
def dis(x1, x2, y1, y2):
	return ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5
            
    
# 이미지에서 얼굴 탐지 역할
detector = dlib.get_frontal_face_detector()

# 얼굴에서 랜드마크 찾는 역할
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

# cap에 0번 웹캠 할당, 해상도 조절
cap = cv2.VideoCapture(0)				# cap에 0번 웹캠 할당
cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)	# 웹캠 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)	# 해상도에 따라 웹캠 인식 안되고 오류나는 현상발생

# range는 끝값이 포함안됨 (얼굴 부위의 점 번호)
ALL = list(range(0, 68)) 
RIGHT_EYEBROW = list(range(17, 22))  
LEFT_EYEBROW = list(range(22, 27))  
RIGHT_EYE = list(range(36, 42))  
LEFT_EYE = list(range(42, 48))  
NOSE = list(range(27, 36))  
MOUTH_OUTLINE = list(range(48, 61))  
MOUTH_INNER = list(range(61, 68)) 
JAWLINE = list(range(0, 17)) 

index = ALL

# 직접 설정해야되는 고개를 돌려야 하는 사진의 갯수
DETECT = 7;

# 고개를 돌린 사진의 갯수
detect_num_right = 0
detect_num_left = 0

# 기다리는 시간
wait_time = 0
WAIT = 20

# 왼쪽 오른쪽, 오른쪽 왼쪽 비율( > 0.5)
LEFT_RATIO = 0.8
RIGHT_RATIO = 0.8

# 버튼이 눌렸을 때, 사용자가 ratio값을 조절할 수 있게 한다.
global Button
global default_size
global default_nose_x
default_size = 47
default_nose_x = 80
Button = 0
Buttontmp = 0
detect_botton = 0
BUTTONCOUNT = 6
ratio_sum = 0



def new_left_ratio(size, face_x):
	global default_size
	global default_nose_x
	dis = 30.0
	l = 60.0 /(size) * default_size
	k = (default_nose_x - face_x) * l / 190.0 # 60:15 = 190:47
	ret = (dis+k) / (((dis + k)*(dis + k) + l*l)**0.5)
	ret = (ret+1) /2
	ret += 0.01
	#print('left ratio = ' + str(ret))
	return ret

def new_right_ratio(size, face_x):
	global default_size
	global default_nose_x
	dis = 30.0
	l = 60.0 /(size) * default_size
	k = (default_nose_x - face_x) * l / 190.0
	ret = (dis-k) / (((dis - k)*(dis - k) + l*l)**0.5)
	ret = (ret+1) /2
	ret += 0.01
	#print('right ratio = ' + str(ret))
	return ret

def button_chk(daf, das):
	global Button
	global graph
	buttonin = '1'
	while(1):
		buttonin = input()
		if(buttonin == '0'):
			Button = 1
		elif(buttonin == '2'):
			graph = 1
t = threading.Thread(target=button_chk,args=(1,1))
t.start()
while True:

	# while문 process 시작 타임
	start = time.time()

	ret, img_frame = cap.read()
	img_gray = cv2.cvtColor(img_frame, cv2.COLOR_BGR2GRAY)
	dets = detector(img_gray, 1)

	# 얼굴이 2개 이상, 0개를 찾았을 경우 넘어간다.
	if len(dets) == 0:
		"""print("No face detected");"""
	elif len(dets) >= 2:
		print("face more than 2")
	else:	
		shape = predictor(img_frame, dets[0]) #얼굴에서 68개 점 찾기
		
		# test code
		list_points = []
		for p in shape.parts():
			list_points.append([p.x, p.y])

		list_points = np.array(list_points)

		for i,pt in enumerate(list_points[index]):

			pt_pos = (pt[0], pt[1])
			cv2.circle(img_frame, pt_pos, 2, (0, 255, 0), -1)

		cv2.rectangle(img_frame, (dets[0].left(), dets[0].top()), (dets[0].right(), dets[0].bottom()), (0, 0, 255), 3)
		# left right side detect
        #shape.parts(3).x 호출 할 때 오류 떠서 아래 처럼 수정했습니다
		left_dis  = dis(list_points[3][0], list_points[33][0], list_points[3][1], list_points[33][1])
		right_dis = dis(list_points[13][0], list_points[33][0], list_points[13][1], list_points[33][1])
		#print(left_dis / (left_dis + right_dis))
        #실시간 고개 돌린 비율 출력
		if Button == 1:
			Button = 0;
			Buttontmp = 1
			detect_num_right = 0
			detect_num_left = 0
			wait_time = 0
			print("led_off		       ",end='\r')
			detect_button = 0
			nose_x_sum = 0
			nose_y_sum = 0
			face_size_sum = 0

		if Buttontmp == 1:
			# 왼쪽을 봤을 때 기준으로
			detect_button += 1
			nose_x_sum += (list_points[3][0] + list_points[13][0])/2
			face_size_sum += left_dis + right_dis
			if detect_button == BUTTONCOUNT:
				default_nose_x = nose_x_sum / (float(BUTTONCOUNT) - 1)
				default_size = face_size_sum / (float(BUTTONCOUNT) - 1)
				#print("x,y " + str(default_nose_x) + " "+ str(nose_y))
				#print("face size = "+ str(default_size))
				Buttontmp = 0
        
		####
		RIGHT_RATIO = new_right_ratio(left_dis + right_dis, (list_points[3][0] + list_points[13][0])/2)
		#print('current ratio : ' + str(right_dis / (left_dis + right_dis)))
		LEFT_RATIO = new_left_ratio(left_dis + right_dis, (list_points[3][0] + list_points[13][0])/2)
		#print('current ratio : ' + str(left_dis / (left_dis + right_dis)))
		#print("new "+str(new))
		# 반대방향으로 고개를 돌리는지 체크해주는 곳
		if detect_num_right >= DETECT:
			wait_time += 1
			print("led_on(right)       ",end='\r')

			# 돌렸으면 다음페이지로
			if left_dis / (left_dis + right_dis) > LEFT_RATIO:
				next_page()
				print("next page     ",end='\r')
				wait_time = 0
				detect_num_right = 0
		elif detect_num_left >= DETECT:
			wait_time += 1
			print("led_on(left)      ",end='\r'),
			if right_dis / (left_dis + right_dis) > RIGHT_RATIO:
				prev_page()
				print("previous page      ",end='\r')
				wait_time = 0
				detect_num_left = 0

		# 처음 고개를 돌리는지 체크
		elif left_dis / (left_dis + right_dis) > LEFT_RATIO:
			"""print("left_side  ",left_dis /(left_dis+right_dis))"""
			detect_num_left += 1
			detect_num_right = 0
		elif right_dis / (left_dis + right_dis) > RIGHT_RATIO:
			"""print("right_side ",left_dis /(left_dis+right_dis))"""
			detect_num_right += 1
			detect_num_left = 0
		else:
			"""print("front_side ",left_dis /(left_dis+right_dis))"""
			wait_time = 0
			detect_num_right = 0
			detect_num_left = 0

		# wait time 이 WAIT와 같아지면 초기화
		if wait_time > WAIT:
			print("led_off     ",end='\r')
			wait_time = 0
			detect_num_right = 0
			detect_num_left = 0
		arr_y[int(right_dis*100 / (left_dis + right_dis))] += 1.0
		
	cv2.imshow('result', img_frame)
    #그래프 그림

	key = cv2.waitKey(1)
	if key == 27:
		break
    
	if graph == 1:
		graph = 0
		for i in range(100):
			print(str(int(arr_y[i])))
			f.write(str(int(arr_y[i])))
			f.write('\n')
		f.close()
	# 걸린 시간 출력(0.2초 간격으로 프로세싱)
	#print("processing time: {}".format(time.time() - start))
	while time.time() - start < 0.2 : continue

cap.release()
servo1.stop()
servo2.stop()
servo3.stop()
servo4.stop()
servo5.stop()
GPIO.cleanup()
