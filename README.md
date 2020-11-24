# Pan/tilt face tracking with a Raspberry Pi and OpenCV
        최종 수정  2020년 11월 24일

## 0.환경 설정
본 코드를 작성할때 사용한 하드웨어는 라즈베리파이3 B+, raspberry pi camera, Pimoroni pan tilt HAT full kit 을 사용하였음.  
다른 카메라를 사용하여 제어하려면 코드를 수정해야함.  

###  Enable the i2c interface as well as the camera interface

        $ sudo raspi-config  

### Install pantilthat, imutils

        $ pip install pantilthat  
        $ pip install imutils  

## 1.사용방법
본 repository를 clone한 뒤 src 폴더 내에있는 face_tracker_haarcascade_pid.py를 실행하면됨.  

        $ git clone https://github.com/HY-HIGH/pan_tilt.git  
        $ chmod +x /src/face_tracker_haarcascade_pid.py  
        $ python /src/face_tracker_haarcascade_pid.py  

실행하게되면, Opencv Image 창이뜨고 사람얼굴이 검출되면 박스가 그려짐.  
검출된 박스가 화면 중앙에 오게끔 제어 됨.


[참조 : https://www.pyimagesearch.com/2019/04/01/pan-tilt-face-tracking-with-a-raspberry-pi-and-opencv/]
