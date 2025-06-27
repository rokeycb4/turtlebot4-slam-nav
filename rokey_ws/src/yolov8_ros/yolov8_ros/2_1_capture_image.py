import cv2
import os

# ===============================
# 설정값
# ===============================
SAVE_DIR = "img_capture"     # 이미지 저장 디렉토리
CAPTURE_KEY = 'c'            # 저장 키
EXIT_KEY = 'q'               # 종료 키

# ===============================
# 이미지 캡처 함수
# ===============================
def capture_images():
    # 디렉토리 생성
    os.makedirs(SAVE_DIR, exist_ok=True)

    # 사용자 접두어 입력
    prefix = input("저장할 파일의 접두어를 입력하세요: ").strip()
    file_prefix = f"{prefix}_"
    print(f"\n파일 접두어: {file_prefix}")


    cap = cv2.VideoCapture(0)  #노트북 내장 카메라
    # cap = cv2.VideoCapture(1) #노트북에 연결한 USB 캠
    if not cap.isOpened():
        print("카메라를 열 수 없습니다. 연결 상태를 확인하세요.")
        return

    print("\n[안내] 'c' 키를 누르면 이미지가 저장됩니다.")
    print(f"[안내] '{EXIT_KEY}' 키를 누르면 프로그램이 종료됩니다.\n")

    image_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽지 못했습니다.")
            break

        # 프레임 표시
        cv2.imshow("Webcam", frame)

        # 키 입력 감지
        key = cv2.waitKey(1) & 0xFF

        if key == ord(CAPTURE_KEY):
            filename = os.path.join(SAVE_DIR, f"{file_prefix}img_{image_count}.jpg")
            cv2.imwrite(filename, frame)
            print(f"이미지 저장 완료: {filename}")
            image_count += 1

        elif key == ord(EXIT_KEY):
            print("캡처를 종료합니다.")
            break

    cap.release()
    cv2.destroyAllWindows()

# ===============================
# 실행 진입점
# ===============================
def main():
    capture_images()

if __name__ == "__main__":
    main()
