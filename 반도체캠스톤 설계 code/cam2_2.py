# 패키지 설치
# pip install dlib opencv-python
#
# 학습 모델 다운로드 
# http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
import dlib
import cv2
import numpy as np
import time
import sys

# 두 점 사이의 거리
def dis(x1, x2, y1, y2):
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

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
DETECT = 10;
# 고개를 돌린 사진의 갯수
detect_num_right = 0;
detect_num_left = 0;
# 왼쪽 오른쪽, 오른쪽 왼쪽 비율( > 0.5)
RATIO = 0.75;

while True:

	# while문 process 시작 타임
	start = time.time()

	ret, img_frame = cap.read()
	img_gray = cv2.cvtColor(img_frame, cv2.COLOR_BGR2GRAY)
	dets = detector(img_gray, 1)

	# 얼굴이 2개 이상, 0개를 찾았을 경우 넘어간다.
	if len(dets) == 0:
		print("No face detected");
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
		left_dis  = dis(list_points[3][0], list_points[33][0], list_points[3][1], list_points[33][1]);
		right_dis = dis(list_points[13][0], list_points[33][0], list_points[13][1], list_points[33][1]);
        #실시간 고개 돌린 비율 출력
		if left_dis * (1 - RATIO) > right_dis * RATIO:
			print("left_side  ",left_dis /(left_dis+right_dis))
			detect_num_left += 1
			detect_num_right = 0
		elif right_dis * (1 - RATIO) > left_dis * RATIO:
			print("right_side ",left_dis /(left_dis+right_dis))
			detect_num_right += 1
			detect_num_left = 0
		else:
			print("front_side ",left_dis /(left_dis+right_dis))
			detect_num_right = 0
			detect_num_left = 0

		

	cv2.imshow('result', img_frame)

	key = cv2.waitKey(1)
	if key == 27:
		break
    
	# 약 1초간 고개를 돌리면 페이지를 넘겨줌
	if detect_num_right == DETECT:
		print("next page")
		detect_num_right = 0;
	elif detect_num_left == DETECT:
		print("previous page")
		detect_num_left = 0;

	# 걸린 시간 출력(0.2초 간격으로 프로세싱)
	#print("processing time: {}".format(time.time() - start))
	while time.time() - start < 0.2 : continue

cap.release()
