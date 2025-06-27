# yolov8_ros/yolo_basic_test.py

from ultralytics import YOLO

def main():
    model = YOLO('yolov8n.pt')  # Nano 모델 로드
    model.predict(source='https://ultralytics.com/images/bus.jpg', save=True)

if __name__ == '__main__':
    main()
