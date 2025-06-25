import cv2
import threading
import time

# =======================
# 전역 변수 정의
# =======================
shared_frame = None
lock = threading.Lock()
is_running = True

# =======================
# 스레드: 웹캠 프레임 캡처
# =======================
def webcam_capture_loop():
    global shared_frame, is_running

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        is_running = False
        return

    while is_running:
        ret, frame = cap.read()
        if ret:
            with lock:
                shared_frame = frame.copy()
        time.sleep(0.001)

    cap.release()

# =======================
# 무거운 처리 (예: GaussianBlur 반복)
# =======================
def heavy_processing(frame):
    for _ in range(10):
        frame = cv2.GaussianBlur(frame, (9, 9), 0)
    return frame

# =======================
# 스레드 기반 처리 실행
# =======================
def run_threaded_version(duration=10):
    global is_running
    is_running = True

    # 웹캠 캡처용 스레드 시작
    capture_thread = threading.Thread(target=webcam_capture_loop)
    capture_thread.start()

    print(f"Running THREADED version for {duration} seconds...")
    frame_count = 0
    start_time = time.time()

    while time.time() - start_time < duration:
        with lock:
            frame = shared_frame.copy() if shared_frame is not None else None

        if frame is not None:
            processed = heavy_processing(frame)
            cv2.imshow("Threaded", processed)
            frame_count += 1

        if cv2.waitKey(1) == 27:  # ESC 키로 중지 가능
            break

    is_running = False
    capture_thread.join()
    cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    print(f"[THREADED] Avg FPS: {frame_count / elapsed:.2f}")

# =======================
# 일반 (비스레드) 처리 실행
# =======================
def run_inline_version(duration=10):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open webcam.")
        return

    print(f"Running INLINE version for {duration} seconds...")
    frame_count = 0
    start_time = time.time()

    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if not ret:
            continue

        processed = heavy_processing(frame)
        cv2.imshow("Inline", processed)
        frame_count += 1

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    print(f"[INLINE] Avg FPS: {frame_count / elapsed:.2f}")

# =======================
# 실행 진입점
# =======================
if __name__ == "__main__":
    mode = input("Enter 't' for threaded, anything else for inline: ").strip().lower()

    if mode == 't':
        run_threaded_version()
    else:
        run_inline_version()
