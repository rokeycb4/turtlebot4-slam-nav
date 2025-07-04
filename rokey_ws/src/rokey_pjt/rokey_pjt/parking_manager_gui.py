import sys
import os
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QStackedWidget,
                             QFileDialog, QMessageBox)
from PyQt5.QtGui import QPixmap, QColor, QPainter, QBrush, QPen, QFont, QImage
from PyQt5.QtCore import Qt, QSize, QRect, QTimer
import traceback

# YOLOv8 모델 로드를 위한 ultralytics 라이브러리
from ultralytics import YOLO

# Qt 플러그인 경로 문제 해결을 위한 환경 변수 설정
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins"

# 카메라 번호 설정
camera1 = 4 # cctv1
camera2 = 2 # cctv2

class ParkingManagerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_camera = 0  # 첫 번째 카메라
        self.current_camera2 = 0  # 두 번째 카메라
        self.dir = os.path.dirname(os.path.abspath(__file__))
        self.yolo_path = os.path.join(self.dir, "model")
        self.yolo_model_file = os.path.join(self.yolo_path, "car_detect.pt")
        self.yolo_model = YOLO(self.yolo_model_file)  # YOLO 모델
        self.yolo_conf = 0.85 # 신뢰도 임계값
        self.yolo_classes = ['cars'] # 클래스: 'cars'
        self.initUI()
        
    def initUI(self):
        # 메인 윈도우 설정
        self.setWindowTitle('로봇 자동화 주차 시스템')
        self.setGeometry(100, 100, 400, 700)
        self.setStyleSheet("background-color: #2D2B43;")
        
        # 중앙 위젯 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 스택 위젯 (탭 전환용)
        self.stack = QStackedWidget()
        
        # 메인 시작 화면
        self.main_screen = self.create_main_screen()
        self.stack.addWidget(self.main_screen)
        
        # 주차 관리 탭
        self.parking_tab = self.create_parking_tab()
        self.stack.addWidget(self.parking_tab)
        
        # 차량 감지 탭
        self.detection_tab = self.create_detection_tab()
        self.stack.addWidget(self.detection_tab)
        
        # 스택 위젯 추가
        main_layout.addWidget(self.stack)
        
        # 하단 버튼 영역
        self.button_area = QWidget()
        self.button_area.setFixedHeight(80)
        self.button_area.setStyleSheet("background-color: #3E3B5A;")
        button_layout = QHBoxLayout(self.button_area)
        
        # 주차 관리 버튼
        self.parking_btn = QPushButton()
        self.parking_btn.setFixedSize(100, 60)
        self.parking_btn.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                border: none;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #4B4870;
                border: 2px outset #6A6A8E;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #3A3A5A;
                border: 2px inset #555577;
                border-radius: 10px;
            }
        """)
        self.parking_btn.clicked.connect(lambda: self.switch_tab(1))
        
        # 주차 관리 버튼 레이아웃
        parking_btn_layout = QVBoxLayout()
        parking_icon = QLabel()
        parking_icon.setAlignment(Qt.AlignCenter)
        # 아이콘 추가
        parking_icon.setText("👤")
        parking_icon.setStyleSheet("color: #8FBC8F; font-size: 24px;")
        
        parking_text = QLabel("주차장 관리")
        parking_text.setAlignment(Qt.AlignCenter)
        parking_text.setStyleSheet("color: #8FBC8F;")
        
        parking_btn_layout.addWidget(parking_icon)
        parking_btn_layout.addWidget(parking_text)
        self.parking_btn.setLayout(parking_btn_layout)
        
        # 차량 감지 버튼
        self.detection_btn = QPushButton()
        self.detection_btn.setFixedSize(100, 60)
        self.detection_btn.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                border: none;
                color: white;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #4B4870;
                border: 2px outset #6A6A8E;
                border-radius: 10px;
            }
            QPushButton:pressed {
                background-color: #3A3A5A;
                border: 2px inset #555577;
                border-radius: 10px;
            }
        """)
        self.detection_btn.clicked.connect(lambda: self.switch_tab(2))
        
        # 차량 감지 버튼 레이아웃
        detection_btn_layout = QVBoxLayout()
        detection_icon = QLabel()
        detection_icon.setAlignment(Qt.AlignCenter)
        # 아이콘 추가
        detection_icon.setText("🚗")
        detection_icon.setStyleSheet("color: #8FBC8F; font-size: 24px;")
        
        detection_text = QLabel("차량 감지")
        detection_text.setAlignment(Qt.AlignCenter)
        detection_text.setStyleSheet("color: #8FBC8F;")
        
        detection_btn_layout.addWidget(detection_icon)
        detection_btn_layout.addWidget(detection_text)
        self.detection_btn.setLayout(detection_btn_layout)
        
        # 버튼 추가
        button_layout.addWidget(self.parking_btn)
        button_layout.addWidget(self.detection_btn)
        
        # 하단 버튼 영역 추가
        main_layout.addWidget(self.button_area)
        
        # 초기 탭 설정
        self.switch_tab(0)
        
        # 카메라 초기화
        self.initCamera()
        
    def create_main_screen(self):
        """메인 시작 화면 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)
        
        # 상단 여백
        layout.addSpacing(60)
        
        # 로고 영역
        logo_area = QWidget()
        logo_area.setFixedHeight(150)
        logo_area.setStyleSheet("background-color: #2D2B43;")
        logo_layout = QVBoxLayout(logo_area)
        
        # 로봇 아이콘
        robot_icon = QLabel()
        robot_icon.setAlignment(Qt.AlignCenter)
        robot_icon.setText("🤖")
        robot_icon.setStyleSheet("color: #8FBC8F; font-size: 40px;")
        logo_layout.addWidget(robot_icon)
        
        # 시스템 제목
        title_label = QLabel("로봇 자동화")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: white; font-size: 28px; font-weight: bold;")
        logo_layout.addWidget(title_label)
        
        # 시스템 부제목
        subtitle_label = QLabel("주차 시스템 관리")
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setStyleSheet("color: white; font-size: 28px; font-weight: bold;")
        logo_layout.addWidget(subtitle_label)
        
        layout.addWidget(logo_area)
        layout.addSpacing(100)
        
        # 주차하기 버튼
        parking_button = QPushButton("👤  주차장 관리")
        parking_button.setFixedHeight(60)
        parking_button.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4B72B0, stop:1 #6A98E0);
                border-radius: 30px;
                color: white;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #5A81BF, stop:1 #79A7EF);
            }
            QPushButton:pressed {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #3C63A1, stop:1 #5B89D1);
            }
        """)
        parking_button.clicked.connect(lambda: self.switch_tab(1))
        layout.addWidget(parking_button)
        
        # 출차하기 버튼
        exit_button = QPushButton("🚗  차량감지")
        exit_button.setFixedHeight(60)
        exit_button.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2AAFDE, stop:1 #6A7FE0);
                border-radius: 30px;
                color: white;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #39BEEF, stop:1 #798EEF);
            }
            QPushButton:pressed {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1B9FCF, stop:1 #5B70D1);
            }
        """)
        exit_button.clicked.connect(lambda: self.switch_tab(2))
        layout.addWidget(exit_button)
        
        # 하단 여백
        layout.addStretch()
        
        # 버전 정보
        version_label = QLabel("v1.0.0")
        version_label.setAlignment(Qt.AlignCenter)
        version_label.setStyleSheet("color: #8FBC8F; font-size: 12px;")
        layout.addWidget(version_label)
        
        return tab
        
    def initCamera(self):
        """카메라 초기화"""
        # 기존 카메라가 있으면 해제
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            
        if hasattr(self, 'cap2') and self.cap2 is not None:
            self.cap2.release()
            
        # 타이머가 있으면 중지
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.stop()
            
        # 카메라 3번 연결
        self.current_camera = camera1
        print(f"cctv1 카메라 연결 중...")
        self.cap = cv2.VideoCapture(self.current_camera)
        
        # 카메라 5번 연결
        self.current_camera2 = camera2
        print(f"cctv2 카메라 연결 중...")
        self.cap2 = cv2.VideoCapture(self.current_camera2)
    
        if not self.cap.isOpened():
            # 상태 메시지 표시
            self.parking_cctv1_view.setText("No Camera")
            self.detection_cctv1_view.setText("No Camera")
            
        if not self.cap2.isOpened():
            # 상태 메시지 표시
            self.parking_cctv2_view.setText("No Camera")
            self.detection_cctv2_view.setText("No Camera")
            return False
        
        # 타이머 설정 (30ms마다 프레임 업데이트 - 약 30fps)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
        return True
    
    def update_frame(self):
        """웹캠 프레임 업데이트"""
        # 메인 화면일 경우 카메라 업데이트 안함
        current_tab = self.stack.currentIndex()
        if current_tab == 0:
            return
            
        # cctv1 카메라 연결
        ret, frame = self.cap.read()
        if ret:
            # 현재 탭에 따라 처리
            
            # 차량 감지 탭이고 YOLO 모델이 로드된 경우
            if current_tab == 2 and self.yolo_model is not None:
                detection_frame = self.detect_vehicles(frame, "cctv1")
            else:
                detection_frame = frame
            
            # OpenCV는 BGR 형식, PyQt는 RGB 형식을 사용하므로 변환
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detection_frame_rgb = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2RGB)
            
            # 프레임 크기 조정
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            
            # QImage로 변환
            qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            qt_detection_image = QImage(detection_frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # 현재 탭에 따라 적절한 화면에 표시
            if current_tab == 1:  # 주차 관리 탭
                self.parking_cctv1_view.setPixmap(QPixmap.fromImage(qt_image).scaled(
                    self.parking_cctv1_view.width(), 
                    self.parking_cctv1_view.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
            elif current_tab == 2:  # 차량 감지 탭
                self.detection_cctv1_view.setPixmap(QPixmap.fromImage(qt_detection_image).scaled(
                    self.detection_cctv1_view.width(), 
                    self.detection_cctv1_view.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
                
        # cctv2 카메라 연결
        ret2, frame2 = self.cap2.read()
        if ret2:
            # 현재 탭에 따라 처리
            
            # 차량 감지 탭이고 YOLO 모델이 로드된 경우
            if current_tab == 2 and self.yolo_model is not None:
                detection_frame2 = self.detect_vehicles(frame2, "cctv2")
            else:
                detection_frame2 = frame2
            
            # OpenCV는 BGR 형식, PyQt는 RGB 형식을 사용하므로 변환
            frame2_rgb = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)
            detection_frame2_rgb = cv2.cvtColor(detection_frame2, cv2.COLOR_BGR2RGB)
            
            # 프레임 크기 조정
            h, w, ch = frame2_rgb.shape
            bytes_per_line = ch * w
            
            # QImage로 변환
            qt_image2 = QImage(frame2_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            qt_detection_image2 = QImage(detection_frame2_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # 현재 탭에 따라 적절한 화면에 표시
            if current_tab == 1:  # 주차 관리 탭
                self.parking_cctv2_view.setPixmap(QPixmap.fromImage(qt_image2).scaled(
                    self.parking_cctv2_view.width(), 
                    self.parking_cctv2_view.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
            elif current_tab == 2:  # 차량 감지 탭
                self.detection_cctv2_view.setPixmap(QPixmap.fromImage(qt_detection_image2).scaled(
                    self.detection_cctv2_view.width(), 
                    self.detection_cctv2_view.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                ))
    
    def detect_vehicles(self, frame, camera_name):
        """YOLO를 사용하여 차량 감지"""
        if self.yolo_model is None:
            return frame
        
        try:
            # 원본 프레임 복사
            result_frame = frame.copy()
            
            # YOLOv11n 모델로 예측
            results = self.yolo_model(frame, conf=self.yolo_conf, verbose=False)
            
            # 결과 처리
            if len(results) > 0:
                boxes = results[0].boxes
                
                # 감지된 차량 수 카운트
                detected_count = 0
                
                # 감지된 객체가 있는지 확인
                if len(boxes) > 0:
                    for box in boxes:
                        try:
                            # 바운딩 박스 좌표
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            
                            # 바운딩 박스 크기 계산
                            box_width = x2 - x1
                            box_height = y2 - y1
                            frame_height, frame_width = frame.shape[:2]
                            
                            # 화면의 70% 이상을 차지하는 박스는 필터링 (너무 큰 박스는 무시)
                            if (box_width * box_height) > (frame_width * frame_height * 0.7):
                                continue
                                
                            # 너무 작은 박스도 필터링 (노이즈 제거)
                            if (box_width * box_height) < (frame_width * frame_height * 0.01):
                                continue
                                
                            # 신뢰도 점수
                            conf = float(box.conf.item())
                            
                            # 바운딩 박스 그리기
                            cv2.rectangle(result_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            
                            # 클래스 이름 (단일 클래스 'cars'로 고정)
                            cls_name = self.yolo_classes[0]
                            
                            # 레이블 표시 (클래스명과 신뢰도)
                            label = f"{cls_name}: {conf:.2f}"
                            cv2.putText(result_frame, label, (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # 감지된 차량 카운트 증가
                            detected_count += 1
                            
                            # 감지된 객체 로그 출력
                            print(f"[{camera_name}] {cls_name}, 신뢰도: {conf:.2f}, 위치: ({x1},{y1})-({x2},{y2})")
                            
                        except Exception as e:
                            print(f"[{camera_name}] 객체 처리 중 오류: {str(e)}")
                            continue
                
                # 감지된 차량 수 표시
                if detected_count > 0:
                    cv2.putText(result_frame, f"Detected: {detected_count}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(result_frame, "No detected", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            return result_frame
            
        except Exception as e:
            print(f"[{camera_name}] 객체 감지 중 오류 발생: {str(e)}")
            traceback_info = traceback.format_exc()
            print(traceback_info)
            return frame
    
    def create_parking_tab(self):
        """주차 관리 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # CCTV 1 라벨
        cctv1_label = QPushButton("cctv1")
        cctv1_label.setFixedHeight(30)
        cctv1_label.setStyleSheet("""
            background-color: #4B4870;
            color: white;
            border-radius: 15px;
            font-weight: bold;
        """)
        layout.addWidget(cctv1_label)
        
        # CCTV 1 화면
        self.parking_cctv1_view = QLabel()
        self.parking_cctv1_view.setFixedHeight(250)  # 높이 증가
        self.parking_cctv1_view.setStyleSheet("""
            background-color: #2D2B43;
            border-radius: 15px;
            color: white;
            font-size: 16px;
        """)
        self.parking_cctv1_view.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.parking_cctv1_view)
        layout.addSpacing(10)
        
        # CCTV 2 라벨
        cctv2_label = QPushButton("cctv2")
        cctv2_label.setFixedHeight(30)
        cctv2_label.setStyleSheet("""
            background-color: #4B4870;
            color: white;
            border-radius: 15px;
            font-weight: bold;
        """)
        layout.addWidget(cctv2_label)
        
        # CCTV 2 화면
        self.parking_cctv2_view = QLabel()
        self.parking_cctv2_view.setFixedHeight(250)  # 높이 증가
        self.parking_cctv2_view.setStyleSheet("""
            background-color: #2D2B43;
            border-radius: 15px;
            color: white;
            font-size: 16px;
        """)
        self.parking_cctv2_view.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.parking_cctv2_view)
        
        return tab
    
    def create_detection_tab(self):
        """차량 감지 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # CCTV 1 라벨
        cctv1_label = QPushButton("cctv1")
        cctv1_label.setFixedHeight(30)
        cctv1_label.setStyleSheet("""
            background-color: #4B4870;
            color: white;
            border-radius: 15px;
            font-weight: bold;
        """)
        layout.addWidget(cctv1_label)
        
        # CCTV 1 화면
        self.detection_cctv1_view = QLabel()
        self.detection_cctv1_view.setFixedHeight(250)  # 높이 증가
        self.detection_cctv1_view.setStyleSheet("""
            background-color: #2D2B43;
            border-radius: 15px;
            color: white;
            font-size: 16px;
        """)
        self.detection_cctv1_view.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.detection_cctv1_view)
        layout.addSpacing(10)
        
        # CCTV 2 라벨
        cctv2_label = QPushButton("cctv2")
        cctv2_label.setFixedHeight(30)
        cctv2_label.setStyleSheet("""
            background-color: #4B4870;
            color: white;
            border-radius: 15px;
            font-weight: bold;
        """)
        layout.addWidget(cctv2_label)
        
        # CCTV 2 화면
        self.detection_cctv2_view = QLabel()
        self.detection_cctv2_view.setFixedHeight(250)  # 높이 증가
        self.detection_cctv2_view.setStyleSheet("""
            background-color: #2D2B43;
            border-radius: 15px;
            color: white;
            font-size: 16px;
        """)
        self.detection_cctv2_view.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.detection_cctv2_view)
        
        return tab
    
    def switch_tab(self, index):
        """탭 전환 함수"""
        self.stack.setCurrentIndex(index)
        
        # 메인 화면일 경우 하단 탭 버튼 숨기기, 아닐 경우 보이기
        if index == 0:  # 메인 화면
            self.button_area.hide()
        else:
            self.button_area.show()
        
        # 버튼 스타일 업데이트
        if index == 0:  # 메인 화면
            self.parking_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    color: white;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #4B4870;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
                QPushButton:pressed {
                    background-color: #3A3A5A;
                    border: 2px inset #555577;
                    border-radius: 10px;
                }
            """)
            self.detection_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    color: white;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #4B4870;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
                QPushButton:pressed {
                    background-color: #3A3A5A;
                    border: 2px inset #555577;
                    border-radius: 10px;
                }
            """)
        elif index == 1:  # 주차 관리 탭
            self.parking_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4B4870;
                    border: 2px inset #555577;
                    border-radius: 10px;
                    color: #8FBC8F;
                }
                QPushButton:hover {
                    background-color: #5A587F;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
            """)
            self.detection_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    color: white;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #4B4870;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
                QPushButton:pressed {
                    background-color: #3A3A5A;
                    border: 2px inset #555577;
                    border-radius: 10px;
                }
            """)
        else:  # 차량 감지 탭
            self.parking_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    border: none;
                    color: white;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #4B4870;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
                QPushButton:pressed {
                    background-color: #3A3A5A;
                    border: 2px inset #555577;
                    border-radius: 10px;
                }
            """)
            self.detection_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4B4870;
                    border: 2px inset #555577;
                    border-radius: 10px;
                    color: #8FBC8F;
                }
                QPushButton:hover {
                    background-color: #5A587F;
                    border: 2px outset #6A6A8E;
                    border-radius: 10px;
                }
            """)
    
    def closeEvent(self, event):
        """앱 종료 시 리소스 해제"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        if hasattr(self, 'cap2') and self.cap2 is not None:
            self.cap2.release()
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.stop()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = ParkingManagerGUI()
    window.show()
    return app.exec_()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ParkingManagerGUI()
    window.show()
    sys.exit(app.exec_())
