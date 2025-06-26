from ultralytics import YOLO
import cv2
import time  # 시간 측정용 모듈# 모델 로드
model = YOLO('/home/fred/rokey_ws/detect_mu3.pt')# 이미지 로드
img_path = '/home/fred/rokey_ws/images/아군3.png'
image = cv2.imread(img_path)# 추론 시간 측정 시작
start = time.time()
results = model(image)[0]
end = time.time()
elapsed = end - start
print(f"YOLOv11 추론 시간: {elapsed:.4f}초")# 클래스 이름 확인
names = model.names
print(f"클래스 목록: {names}")# 결과 출력 및 시각화
for box in results.boxes:
    cls_id = int(box.cls[0])
    conf = float(box.conf[0])
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    label = names[cls_id] if names else str(cls_id)
    print(f"[{label}] {conf:.2f} → x1={x1}, y1={y1}, x2={x2}, y2={y2}")    
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(image, f'{label} {conf:.2f}', (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)# 결과 이미지 표시
cv2.imshow("YOLOv8 Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()