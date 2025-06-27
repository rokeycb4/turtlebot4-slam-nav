import cv2
import os
import time

# ===============================
# 설정값
# ===============================
SAVE_DIR = "img_capture_1"   # 저장 폴더 (현재 디렉토리 하위)
CAPTURE_INTERVAL = 1.0       # 이미지 캡처 간격 (초 단위)

# ===============================
# 이미지 캡처 함수
# ===============================
def capture_images():
    os.makedirs(SAVE_DIR, exist_ok=True)

    # 사용자로부터 파일 접두어 입력
    prefix_input = input("저장할 파일의 접두어를 입력하세요: ").strip()
    file_prefix = f"{prefix_input}_"

    # 웹캠 열기
    cap = cv2.VideoCapture(0)  #노트북 내장 카메라
    # cap = cv2.VideoCapture(1) #노트북에 연결한 USB 캠
    if not cap.isOpened():
        print("카메라를 열 수 없습니다. 연결을 확인하세요.")
        return

    print("\n[안내] 'q'를 누르면 종료됩니다.\n")

    image_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽지 못했습니다.")
            break

        cv2.imshow("Webcam", frame)
        key = cv2.waitKey(1)

        if key == ord('q'):
            print("캡처를 종료합니다.")
            break
        else:
            filename = os.path.join(SAVE_DIR, f"{file_prefix}img_{image_count}.jpg")
            cv2.imwrite(filename, frame)
            print(f"이미지 저장 완료: {filename}")
            image_count += 1
            time.sleep(CAPTURE_INTERVAL)

    cap.release()
    cv2.destroyAllWindows()


def main():
    capture_images()

if __name__ == "__main__":
    main()
