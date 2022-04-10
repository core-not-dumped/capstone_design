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
  
detector = dlib.get_frontal_face_detector()#이미지에서 얼굴 탐지 역할
 
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')#얼굴에서 랜드마크 찾는 역할


cap = cv2.VideoCapture(0)#cap에 0번 웹캠 할당
cap.set(cv2.CAP_PROP_FRAME_WIDTH,160)#웹캠 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,120)#해상도에 따라 웹캠 인식 안되고 오류나는 현상발생

# range는 끝값이 포함안됨   
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

while True:
    start = time.time()
    

    ret, img_frame = cap.read()

    img_gray = cv2.cvtColor(img_frame, cv2.COLOR_BGR2GRAY)

    dets = detector(img_gray, 1)

    for face in dets:

        shape = predictor(img_frame, face) #얼굴에서 68개 점 찾기

        list_points = []
        for p in shape.parts():
            list_points.append([p.x, p.y])

        list_points = np.array(list_points)


        for i,pt in enumerate(list_points[index]):

            pt_pos = (pt[0], pt[1])
            cv2.circle(img_frame, pt_pos, 2, (0, 255, 0), -1)

        
        cv2.rectangle(img_frame, (face.left(), face.top()), (face.right(), face.bottom()),
            (0, 0, 255), 3)


    cv2.imshow('result', img_frame)

    key = cv2.waitKey(1)
    if key == 27:
        break
    
    print(time.time() - start)

cap.release()
