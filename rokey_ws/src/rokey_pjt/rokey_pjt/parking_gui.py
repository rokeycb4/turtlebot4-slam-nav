#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import pandas as pd
from datetime import datetime, timedelta
import threading
import time
import logging
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QStackedWidget,
                             QMessageBox, QDialog, QComboBox, QListWidget, QListWidgetItem)
from PyQt5.QtGui import QPixmap, QFont, QColor, QPalette, QImage
from PyQt5.QtCore import Qt, QSize, pyqtSignal, QObject, QTimer
import re
import json

# DB 관리자 임포트
from rokey_pjt.db_manager import DBManager

# 로거 설정
logger = logging.getLogger('parking_gui')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# ROS2 관련 import
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

# Qt 플러그인 경로 문제 해결을 위한 환경 변수 설정
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins"

# 토픽 메시지 시뮬레이션을 위한 클래스
class ParkingDataManager(QObject):
    # 데이터 업데이트 시그널 정의
    data_updated = pyqtSignal(dict)
    # 차량 위치 정보 시그널 정의
    parking_location_updated = pyqtSignal(str)  # 차량 위치 정보
    # 카메라 영상 시그널 정의
    camera_frame_updated = pyqtSignal(QImage)  # 카메라 프레임 업데이트
    # OCR 결과 업데이트 시그널 정의
    ocr_result_updated = pyqtSignal(str, str)  # 번호판, 차량 타입
    
    def __init__(self):
        super().__init__()
        
        # DB 관리자 초기화
        self.db_manager = DBManager()
        logger.info("DB 관리자 초기화 완료")
        
        # 주차장 상태 정보 초기화 - 사용중 0, 주차 가능 2로 설정
        self.parking_data = {
            "total": {"normal": 2, "ev": 2, "disabled": 2},
            "occupied": {"normal": 0, "ev": 0, "disabled": 0},
            "available": {"normal": 2, "ev": 2, "disabled": 2}
        }
        
        # 주차된 차량 정보 저장 (번호판 -> {차량 타입, 차량 위치})
        self.parked_vehicles = {}
        
        # 주차 이력 정보 저장
        self.parking_history = []
        
        # 주차 위치 관리 (차량 타입별로 사용 가능한 위치 관리)
        self.parking_locations = {
            "normal": ["A-1", "A-2"],  # 일반 차량 위치
            "ev": ["B-1", "B-2"],      # 전기 차량 위치
            "disabled": ["C-1", "C-2"] # 장애인 차량 위치
        }
        
        # 사용 중인 주차 위치 (위치 -> 차량 번호)
        self.occupied_locations = {}
        
        # 현재 차량 위치 정보
        self.current_parking_location = "-"
        
        # 데이터 로드
        self.load_data()
        
        # 주기적 데이터 갱신을 위한 타이머 설정
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.refresh_data)
        self.update_timer.start(5000)  # 5초마다 데이터 갱신 (기존 10초에서 단축)
        
        # 마지막 데이터 갱신 시간
        self.last_refresh_time = datetime.now()
        
        # ROS2 관련 변수
        self.ros_node = None
        self.bridge = CvBridge()
        self.camera_subscription = None
        self.ros_thread = None
        self.running = True

        # 토픽 변수
        # self.camera_sub = '/robot2/oakd/rgb/image_raw'   # --> 2번 turtlebot 
        # self.camera_sub = '/robot3/oakd/rgb/image_raw'   # --> 3번 turtlebot
        self.camera_sub = '/detect/yolo_distance_image' # --> 거리 감지 영상 토픽
        self.ocr_sub = '/carplate/ocr_result' # --> 번호판 OCR 결과 토픽
        self.location_pub = '/parking/location' # --> 주차 위치 발행 토픽
        
        # ROS2 초기화 및 토픽 구독 시작
        self.start_ros()
    
    def refresh_data(self):
        """주기적으로 데이터베이스에서 데이터 갱신"""
        try:
            current_time = datetime.now()
            elapsed_seconds = (current_time - self.last_refresh_time).total_seconds()
            
            # 너무 빈번한 갱신 방지 (최소 5초 간격)
            if elapsed_seconds < 5:
                return
                
            logger.info("데이터베이스에서 데이터 갱신 중...")
            self.last_refresh_time = current_time
            
            # 1. 현재 주차된 차량 정보만 빠르게 조회    
            parked_df = self.db_manager.fetch_current_parked_vehicles()
            
            # 주차된 차량 정보 업데이트
            self.parked_vehicles = {}
            self.occupied_locations = {}
            
            if not parked_df.empty:
                for _, row in parked_df.iterrows():
                    license_plate = row['license_plate']
                    car_type = row['car_type']
                    location = row['location']
                    
                    # 주차된 차량 정보 저장
                    self.parked_vehicles[license_plate] = {
                        'car_type': car_type,
                        'location': location
                    }
                    
                    # 사용 중인 위치 정보 업데이트
                    if location != '-':
                        self.occupied_locations[location] = license_plate
            
            # 2. 주차장 통계 정보 업데이트 (DB 쿼리 없이 로컬에서 계산)
            self.update_statistics()
            
            # 3. 데이터 업데이트 시그널 발생
            self.data_updated.emit(self.parking_data)
            
            logger.info(f"데이터 갱신 완료: {len(self.parked_vehicles)}대 주차 중")
        except Exception as e:
            logger.error(f"데이터 갱신 중 오류 발생: {e}")
    
    def update_statistics(self):
        """주차된 차량 정보를 바탕으로 통계 정보 업데이트"""
        # 기본값으로 초기화
        self.parking_data = {
            "total": {"normal": 2, "ev": 2, "disabled": 2},
            "occupied": {"normal": 0, "ev": 0, "disabled": 0},
            "available": {"normal": 2, "ev": 2, "disabled": 2}
        }
        
        # 주차된 차량 정보를 바탕으로 통계 계산
        for _, info in self.parked_vehicles.items():
            car_type = info.get('car_type', 'normal')
            if car_type in self.parking_data["occupied"]:
                self.parking_data["occupied"][car_type] += 1
                self.parking_data["available"][car_type] = max(0, self.parking_data["total"][car_type] - self.parking_data["occupied"][car_type])
    
    def load_data(self):
        """데이터베이스에서 데이터 로드"""
        try:
            logger.info("데이터베이스에서 데이터 로드 중...")
            
            # 1. 현재 주차된 차량 조회 (최적화된 쿼리 사용)
            parked_df = self.db_manager.fetch_current_parked_vehicles()
            
            # 주차된 차량 정보 초기화
            self.parked_vehicles = {}
            self.occupied_locations = {}
            
            # 주차된 차량 정보 업데이트
            if not parked_df.empty:
                for _, row in parked_df.iterrows():
                    license_plate = row['license_plate']
                    car_type = row['car_type']
                    location = row['location']
                    
                    # 주차된 차량 정보 저장
                    self.parked_vehicles[license_plate] = {
                        'car_type': car_type,
                        'location': location
                    }
                    
                    # 사용 중인 위치 정보 업데이트
                    if location != '-':
                        self.occupied_locations[location] = license_plate
            
            # 2. 최근 출차 기록 조회 (주차 이력용) - 필요할 때만 로드
            # exit_df = self.db_manager.fetch_recent_exit_records()
            
            # 3. 모든 주차 데이터 조회 (필요할 때만 로드)
            all_data = self.db_manager.fetch_all_parking_data()
            
            # 주차 이력 정보 업데이트
            self.parking_history = []
            if all_data is not None and not all_data.empty:
                for _, row in all_data.iterrows():
                    history_data = {
                        "license_plate": row['license_plate'],
                        "car_type": row['car_type'],
                        "location": row['location'],
                        "status": row['status'],
                        "timestamp": row['time']
                    }
                    
                    # 날짜와 시간 정보 추출
                    try:
                        dt = pd.to_datetime(row['time'])
                        history_data["date"] = dt.strftime("%Y-%m-%d")
                        history_data["time"] = dt.strftime("%H:%M:%S")
                        history_data["action"] = "주차" if row['status'] == 'parked' else "출차"
                    except:
                        pass
                    
                    self.parking_history.append(history_data)
            
            # 4. 주차장 통계 정보 업데이트
            self.update_statistics()
            
            logger.info(f"데이터 로드 완료: {len(self.parked_vehicles)}대 주차 중")
            
        except Exception as e:
            logger.error(f"데이터 로드 중 오류 발생: {e}")
            logger.exception("데이터 로드 중 예외 발생")
    
    def start_ros(self):
        """ROS2 초기화 및 토픽 구독 시작"""
        # ROS2 스레드 시작
        self.ros_thread = threading.Thread(target=self._ros_thread_func)
        self.ros_thread.daemon = True
        self.ros_thread.start()
    
    def _ros_thread_func(self):
        """ROS2 스레드 함수"""
        try:
            # ROS2 초기화
            rclpy.init()
            self.ros_node = Node('parking_gui_node')
            
            # 카메라 토픽 구독
            self.camera_subscription = self.ros_node.create_subscription(
                Image,
                self.camera_sub,
                self._camera_callback,
                10)
            
            # 번호판 OCR 결과 토픽 구독
            self.ocr_subscription = self.ros_node.create_subscription(
                String,
                self.ocr_sub,
                self._ocr_result_callback,
                10)
            
            logger.info("ROS2 초기화 및 토픽 구독 시작")
            
            # 주차 위치 정보 발행을 위한 퍼블리셔 생성
            self.location_publisher = self.ros_node.create_publisher(
                String, 
                self.location_pub, 
                10
            )
            logger.info(f"location_publisher 생성 완료: {self.location_publisher}")
            
            # ROS2 스핀
            while self.running and rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                time.sleep(0.01)
            
        except Exception as e:
            logger.error(f"ROS2 초기화 중 오류 발생: {e}")
            logger.exception("ROS2 초기화 중 예외 발생")
            
        finally:
            # ROS2 종료
            if self.ros_node is not None:
                self.ros_node.destroy_node()
            rclpy.shutdown()
            logger.info("ROS2 종료")
    
    def stop_ros(self):
        """ROS2 종료"""
        logger.info("ROS2 종료 시작")
        self.running = False
        
        # 구독 해제
        if hasattr(self, 'camera_subscription') and self.camera_subscription:
            try:
                self.ros_node.destroy_subscription(self.camera_subscription)
                self.camera_subscription = None
                logger.info("카메라 구독 해제 완료")
            except Exception as e:
                logger.warning(f"카메라 구독 해제 중 오류: {e}")
        
        if hasattr(self, 'ocr_subscription') and self.ocr_subscription:
            try:
                self.ros_node.destroy_subscription(self.ocr_subscription)
                self.ocr_subscription = None
                logger.info("OCR 구독 해제 완료")
            except Exception as e:
                logger.warning(f"OCR 구독 해제 중 오류: {e}")
        
        # 발행자 해제
        if hasattr(self, 'location_publisher') and self.location_publisher:
            try:
                self.ros_node.destroy_publisher(self.location_publisher)
                self.location_publisher = None
                logger.info("위치 발행자 해제 완료")
            except Exception as e:
                logger.warning(f"위치 발행자 해제 중 오류: {e}")
        
        # ROS 스레드 종료 대기
        if self.ros_thread is not None:
            try:
                self.ros_thread.join(2.0)  # 최대 2초 대기
                logger.info("ROS 스레드 종료 대기 완료")
            except Exception as e:
                logger.warning(f"ROS 스레드 종료 대기 중 오류: {e}")
        
        # ROS 노드 정리
        if hasattr(self, 'ros_node') and self.ros_node:
            try:
                self.ros_node.destroy_node()
                self.ros_node = None
                logger.info("ROS 노드 정리 완료")
            except Exception as e:
                logger.warning(f"ROS 노드 정리 중 오류: {e}")
        
        # ROS 종료
        try:
            if rclpy.ok():
                rclpy.shutdown()
                logger.info("ROS2 종료 완료")
        except Exception as e:
            logger.warning(f"ROS2 종료 중 오류: {e}")
    
    def _camera_callback(self, msg):
        """카메라 토픽 콜백 함수"""
        try:
            # 종료 중이면 콜백 처리하지 않음
            if not self.running:
                return
                
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # OpenCV 이미지를 QImage로 변환
            height, width, channels = cv_image.shape
            bytes_per_line = channels * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            # 시그널 발생
            self.camera_frame_updated.emit(q_image)
            
        except Exception as e:
            logger.error(f"카메라 콜백 중 오류 발생: {e}")
    
    def _ocr_result_callback(self, msg):
        """OCR 결과 토픽 콜백 함수"""
        try:
            # 종료 중이면 콜백 처리하지 않음
            if not self.running:
                return
                
            # JSON 문자열을 파싱
            ocr_data = json.loads(msg.data)
            
            # 차량 번호와 타입 추출
            car_plate = ocr_data.get('car_plate', '')
            car_type = ocr_data.get('type', 'normal')
            
            # 'electric' 타입을 'ev'로 변환
            if car_type == 'electric':
                car_type = 'ev'
            
            # 유효한 타입인지 확인
            if car_type not in ['normal', 'ev', 'disabled']:
                car_type = 'normal'  # 기본값으로 설정
            
            logger.info(f"OCR 결과 수신: 번호판={car_plate}, 타입={car_type}")
            
            # 이 부분은 GUI 클래스에서 처리할 수 있도록 시그널 추가 필요
            self.ocr_result_updated.emit(car_plate, car_type)
            
        except Exception as e:
            logger.error(f"OCR 결과 처리 중 오류 발생: {e}")
            logger.exception("OCR 결과 처리 중 예외 발생")
    
    def get_current_data(self):
        """현재 주차장 상태 데이터 반환"""
        return self.parking_data

    def get_parking_location(self, car_type):
        """
        차량 타입에 따라 주차 위치 할당
        
        Args:
            car_type (str): 차량 타입 ('normal', 'ev', 'disabled')
            
        Returns:
            str: 할당된 주차 위치 (예: "A-1", "B-2", "C-1")
        """
        # 해당 차량 타입의 주차 위치 목록
        available_locations = self.parking_locations.get(car_type, [])
        
        # 사용 가능한 위치 찾기
        for location in available_locations:
            if location not in self.occupied_locations:
                return location
        
        # 모든 위치가 사용 중이면 "-" 반환
        return "-"
    
    def publish_location(self, location):
        """
        주차 위치 정보를 ROS 토픽으로 발행
        
        Args:
            location (str): 주차 위치 (예: "A-1", "B-2" 등)
        """
        logger.info(f"publish_location 메서드 호출됨: {location}")
        
        # location_publisher 속성이 있는지 확인
        if not hasattr(self, 'location_publisher'):
            logger.error("[ERROR] location_publisher 속성이 없습니다.")
            return
        
        # location_publisher가 None이 아닌지 확인
        if self.location_publisher is None:
            logger.error("[ERROR] location_publisher가 None입니다.")
            return
        
        try:
            # 메시지 생성 및 발행
            msg = String()
            msg.data = location
            self.location_publisher.publish(msg)
            logger.info(f"[ROS Publish 성공] /parking/location: {location}")
        except Exception as e:
            logger.error(f"[ERROR] 메시지 발행 중 오류 발생: {e}")
            logger.exception("메시지 발행 중 예외 발생")
    
    def simulate_parking(self, car_type="normal", license_plate=""):
        """주차 시뮬레이션"""
        # 번호판이 이미 존재하는지 확인
        if license_plate and license_plate in self.parked_vehicles:
            # 이미 주차된 차량이 있는 경우
            return False, "duplicate"
            
        if self.parking_data["available"][car_type] > 0:
            # 차량 타입에 맞는 주차 위치 할당
            location = self.get_parking_location(car_type)
            logger.info(f"할당된 주차 위치: {location}")
            
            # 위치 정보 업데이트
            self.current_parking_location = location
            
            # 차량 위치 정보 업데이트 시그널 발생
            self.parking_location_updated.emit(location)
            
            # 사용 중인 위치에 추가
            if location != "-":
                self.occupied_locations[location] = license_plate
                
                # 주차 위치 정보 발행 (명시적으로 호출)
                logger.info(f"주차 위치 발행 시도: {location}")
                self.publish_location(location)
            else:
                logger.warning("[WARNING] 주차 위치가 '-'이므로 발행하지 않습니다.")
            
            # 차량 정보 저장
            self.parked_vehicles[license_plate] = {
                'car_type': car_type,
                'location': location
            }
            
            # DB에 주차 기록 추가
            success = self.db_manager.park_vehicle(license_plate, car_type, location)
            if not success:
                logger.error(f"DB에 주차 기록 추가 실패: {license_plate}, {car_type}, {location}")
                return False, "db_error"
            
            # 주차 상태 업데이트
            self.parking_data["occupied"][car_type] += 1
            self.parking_data["available"][car_type] -= 1
            
            # 데이터 업데이트 시그널 발생
            self.data_updated.emit(self.parking_data)
            
            # 데이터 갱신
            self.load_data()
            
            return True, ""
        return False, "no_space"
    
    def simulate_leaving(self, license_plate=""):
        """출차 시뮬레이션"""
        # 번호판으로 차량 찾기
        if license_plate and license_plate in self.parked_vehicles:
            vehicle_info = self.parked_vehicles[license_plate]
            
            if isinstance(vehicle_info, dict):
                car_type = vehicle_info.get('car_type', 'normal')
                location = vehicle_info.get('location', '-')
            else:
                # 이전 버전 호환성을 위한 처리
                car_type = vehicle_info
                location = '-'
            
            # 해당 차량 타입의 주차 상태 업데이트
            if self.parking_data["occupied"][car_type] > 0:
                self.parking_data["occupied"][car_type] -= 1
                self.parking_data["available"][car_type] += 1
                
                # 사용 중인 위치에서 제거
                if location != "-" and location in self.occupied_locations:
                    del self.occupied_locations[location]
                
                # DB에 출차 기록 추가
                success = self.db_manager.exit_vehicle(license_plate, car_type, location)
                if not success:
                    logger.error(f"DB에 출차 기록 추가 실패: {license_plate}, {car_type}, {location}")
                    return False, "", ""
                
                # 주차된 차량 목록에서 삭제
                del self.parked_vehicles[license_plate]
                
                # 데이터 업데이트 시그널 발생
                self.data_updated.emit(self.parking_data)
                
                # 데이터 갱신
                self.load_data()
                
                return True, car_type, location
            
        # 번호판 정보가 없는 경우 주차된 차량이 있으면 첫 번째 차량 출차
        elif not license_plate and self.parked_vehicles:
            # 첫 번째 주차된 차량 선택
            first_license_plate = list(self.parked_vehicles.keys())[0]
            vehicle_info = self.parked_vehicles[first_license_plate]
            
            if isinstance(vehicle_info, dict):
                car_type = vehicle_info.get('car_type', 'normal')
                location = vehicle_info.get('location', '-')
            else:
                # 이전 버전 호환성을 위한 처리
                car_type = vehicle_info
                location = '-'
            
            # 해당 차량 타입의 주차 상태 업데이트
            if self.parking_data["occupied"][car_type] > 0:
                self.parking_data["occupied"][car_type] -= 1
                self.parking_data["available"][car_type] += 1
                
                # 사용 중인 위치에서 제거
                if location != "-" and location in self.occupied_locations:
                    del self.occupied_locations[location]
                
                # DB에 출차 기록 추가
                success = self.db_manager.exit_vehicle(first_license_plate, car_type, location)
                if not success:
                    logger.error(f"DB에 출차 기록 추가 실패: {first_license_plate}, {car_type}, {location}")
                    return False, "", ""
                
                # 주차된 차량 목록에서 삭제
                del self.parked_vehicles[first_license_plate]
                
                # 데이터 업데이트 시그널 발생
                self.data_updated.emit(self.parking_data)
                
                # 데이터 갱신
                self.load_data()
                
                return True, car_type, location
        
        return False, "", ""
    
    def get_parked_vehicles(self):
        """주차된 차량 목록 반환"""
        return self.parked_vehicles
    
    def get_vehicle_info(self, license_plate):
        """번호판으로 차량 정보 조회"""
        vehicle_info = self.parked_vehicles.get(license_plate, None)
        
        if vehicle_info is None:
            return None
        
        if isinstance(vehicle_info, dict):
            return vehicle_info
        else:
            # 이전 버전 호환성을 위한 처리
            return {
                'car_type': vehicle_info,
                'location': '-'
            }
    
    def get_vehicle_type(self, license_plate):
        """번호판으로 차량 타입 조회"""
        vehicle_info = self.get_vehicle_info(license_plate)
        if vehicle_info:
            return vehicle_info.get('car_type', '')
        return ""
    
    def get_vehicle_location(self, license_plate):
        """
        차량 번호로 위치 정보 조회
        
        Args:
            license_plate (str): 차량 번호
            
        Returns:
            str: 차량 위치 정보 (예: "A-1", "B-2", "C-1")
        """
        if license_plate in self.parked_vehicles:
            vehicle_info = self.parked_vehicles[license_plate]
            if isinstance(vehicle_info, dict):
                return vehicle_info.get('location', '-')
            else:
                # 이전 버전 호환성을 위한 처리
                return '-'
        return '-'
    
    def set_parking_location(self, location):
        """차량 위치 설정 (토픽에서 수신 또는 수동 설정)"""
        self.current_parking_location = location
        self.parking_location_updated.emit(location)
    
    def get_parking_history(self):
        """주차 이력 반환"""
        return self.parking_history

class ConfirmDialog(QDialog):
    def __init__(self, title, message, car_types=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setFixedSize(450, 300 if car_types else 250)  # 다이얼로그 크기 증가
        self.setStyleSheet("background-color: #3D3B53;")
        
        # 선택된 차량 타입
        self.selected_car_type = "normal"
        
        # 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 메시지 라벨
        msg_label = QLabel(message)
        msg_label.setStyleSheet("color: white; font-size: 18px;")  # 폰트 크기 증가
        msg_label.setAlignment(Qt.AlignCenter)
        msg_label.setWordWrap(True)  # 자동 줄바꿈 활성화
        layout.addWidget(msg_label)
        
        # 차량 타입 선택 콤보박스
        if car_types:
            type_layout = QHBoxLayout()
            type_layout.setContentsMargins(0, 10, 0, 10)
            
            type_label = QLabel("차량 타입:")
            type_label.setStyleSheet("color: white; font-size: 14px;")
            type_layout.addWidget(type_label)
            
            self.type_combo = QComboBox()
            self.type_combo.setStyleSheet("background-color: white; font-size: 14px; padding: 5px;")
            
            # 차량 타입 옵션 추가
            car_type_names = {
                "normal": "일반 차량",
                "ev": "전기 차량",
                "disabled": "장애인 차량"
            }
            
            for car_type, display_name in car_type_names.items():
                self.type_combo.addItem(display_name, car_type)
            
            self.type_combo.currentIndexChanged.connect(self.on_type_changed)
            type_layout.addWidget(self.type_combo)
            
            layout.addLayout(type_layout)
        
        # 버튼 레이아웃
        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(0, 20, 0, 0)
        
        # 확인 버튼
        self.confirm_btn = QPushButton("확인")
        self.confirm_btn.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px;")
        self.confirm_btn.clicked.connect(self.accept)
        
        # 취소 버튼
        self.cancel_btn = QPushButton("취소")
        self.cancel_btn.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 5px 15px;")
        self.cancel_btn.clicked.connect(self.reject)
        
        btn_layout.addWidget(self.confirm_btn)
        btn_layout.addWidget(self.cancel_btn)
        
        layout.addLayout(btn_layout)
    
    def on_type_changed(self, index):
        """차량 타입 선택 변경 시 호출"""
        self.selected_car_type = self.type_combo.itemData(index)
    
    def get_selected_car_type(self):
        """선택된 차량 타입 반환"""
        return self.selected_car_type

class StartScreen(QWidget):
    """시작 화면 클래스"""
    def __init__(self, parent=None):
        super().__init__(parent)
        # 배경 그라데이션 스타일 적용
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #23234a, stop:1 #3d2b6d);
                border-radius: 0px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 60, 0, 30)

        # 타이틀
        title_label = QLabel("🤖\n로봇 자동화\n주차 시스템")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: white; font-size: 44px; font-weight: bold; line-height: 120%; letter-spacing: 1px;")
        layout.addWidget(title_label)

        layout.addSpacing(100)

        # 버튼 영역
        btn_layout = QVBoxLayout()
        btn_layout.setSpacing(30)
        btn_layout.setContentsMargins(40, 0, 40, 0)

        # 주차하기 버튼
        self.parking_btn = QPushButton()
        self.parking_btn.setText("  🅿️  주차하기")
        self.parking_btn.setCursor(Qt.PointingHandCursor)
        self.parking_btn.setMinimumHeight(90)
        self.parking_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #6a5cff, stop:1 #3ad0ff);
                color: white;
                font-size: 32px;
                font-weight: bold;
                border-radius: 28px;
                padding: 10px 0px;
                border: none;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #7c6cff, stop:1 #4ae0ff);
                color: #23234a;
            }
        """)
        btn_layout.addWidget(self.parking_btn)

        # 출차하기 버튼
        self.exit_btn = QPushButton()
        self.exit_btn.setText("  🚗  출차하기")
        self.exit_btn.setCursor(Qt.PointingHandCursor)
        self.exit_btn.setMinimumHeight(90)
        self.exit_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #3ad0ff, stop:1 #6a5cff);
                color: white;
                font-size: 32px;
                font-weight: bold;
                border-radius: 28px;
                padding: 10px 0px;
                border: none;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #4ae0ff, stop:1 #7c6cff);
                color: #23234a;
            }
        """)
        btn_layout.addWidget(self.exit_btn)

        layout.addLayout(btn_layout)
        layout.addStretch(1)

        # 하단 버전 정보
        version_label = QLabel("v1.0.0")
        version_label.setAlignment(Qt.AlignCenter)
        version_label.setStyleSheet("color: #b0b0b0; font-size: 18px; margin-top: 30px;")
        layout.addWidget(version_label)

class ParkingManagementGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 주차장 데이터 관리자 초기화
        self.data_manager = ParkingDataManager()
        self.data_manager.data_updated.connect(self.update_parking_info)
        
        # 차량 위치 정보 시그널 연결
        self.data_manager.parking_location_updated.connect(self.on_parking_location_updated)
        
        # 카메라 프레임 시그널 연결
        self.data_manager.camera_frame_updated.connect(self.on_camera_frame_updated)
        
        # OCR 결과 시그널 연결 추가
        self.data_manager.ocr_result_updated.connect(self.on_ocr_result_updated)
        
        # 파일 변경 감지 시그널 연결 제거
        # self.data_manager.file_changed.connect(self.on_file_changed)
        
        # 현재 차량 정보
        self.current_license_plate = ""
        self.current_car_type = "normal"
        
        # 출차 모드인지 여부
        self.is_exit_mode = False
        
        # 이미지 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.default_image_path = os.path.join(script_dir, "images/parking_dis.jpg")
        self.exit_image_path = os.path.join(script_dir, "images/exit_dis.jpg")
        
        self.initUI()
        
        # 시작 시 주차 상태 업데이트
        self.update_parking_info(self.data_manager.get_current_data())
        
    def __del__(self):
        # 객체 소멸 시 데이터 저장 및 ROS 종료
        logger.info("프로그램 종료 중... ROS 종료")
        # closeEvent에서 이미 데이터를 저장했으므로 여기서는 저장하지 않음
        # self.data_manager.save_data()
        self.data_manager.stop_ros()
    
    def closeEvent(self, event):
        """프로그램 종료 시 호출되는 이벤트"""
        logger.info("프로그램 종료 이벤트...")
        # ROS 종료는 __del__에서 처리
        # self.data_manager.stop_ros()
        # 기본 종료 이벤트 처리
        super().closeEvent(event)
    
    # 차량 위치 정보 업데이트 처리 메서드
    def on_parking_location_updated(self, location):
        """
        차량 위치 정보 업데이트 처리
        
        Args:
            location (str): 차량 위치 정보 (예: "A-1", "B-3" 등)
        """
        # 주차 화면에서만 위치 정보 업데이트
        if self.stack.currentIndex() == 1:  # 주차 화면
            # 주차 패널의 하단 정보 업데이트
            self.update_bottom_info(self.parking_panel, "차량 위치", location)
        
    def initUI(self):
        self.setWindowTitle('로봇 자동화 주차 시스템')
        self.setGeometry(100, 100, 500, 800)
        
        # 메인 위젯 설정
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 전체 레이아웃
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10) 
        
        # 스택 위젯 생성 (화면 전환용)
        self.stack = QStackedWidget()
        
        # 시작 화면 추가
        self.start_screen = StartScreen()
        self.start_screen.parking_btn.clicked.connect(self.start_parking_flow)
        self.start_screen.exit_btn.clicked.connect(self.start_exit_flow)
        
        # 주차 화면 패널
        self.parking_panel = self.create_panel("주차 관리", self.default_image_path, "차량 번호", "-")
        
        # 출차 화면 패널
        self.exit_panel = self.create_panel("출차 관리", self.exit_image_path, "차량 번호", "-")
        
        # 초기 차량 위치 정보 설정
        self.update_bottom_info(self.parking_panel, "차량 위치", self.data_manager.current_parking_location)
        self.update_bottom_info(self.exit_panel, "차량 위치", self.data_manager.current_parking_location)
        
        # 초기 차량 번호 정보 설정
        self.update_bottom_info(self.parking_panel, "차량 번호", "-")
        self.update_bottom_info(self.exit_panel, "차량 번호", "-")
        
        # 스택에 패널 추가
        self.stack.addWidget(self.start_screen)  # 인덱스 0: 시작 화면
        self.stack.addWidget(self.parking_panel)  # 인덱스 1: 주차 화면
        self.stack.addWidget(self.exit_panel)  # 인덱스 2: 출차 화면
        
        # 메인 레이아웃에 스택 추가
        main_layout.addWidget(self.stack)
        
        # 하단 버튼 영역
        button_widget = QWidget()
        button_widget.setStyleSheet("background-color: #6C6C6C;")
        button_widget.setFixedHeight(50)  # 버튼 영역 높이 고정
        button_layout = QHBoxLayout(button_widget)
        button_layout.setContentsMargins(5, 5, 5, 5)
        
        # 주차 버튼
        self.parking_btn = QPushButton("주차")
        self.parking_btn.setStyleSheet("background-color: white; font-size: 16px; font-weight: bold;")  # 초기 스타일 변경
        self.parking_btn.setFixedHeight(40)
        self.parking_btn.clicked.connect(self.show_parking_confirm)
        
        # 출차 버튼
        self.exit_btn = QPushButton("출차")
        self.exit_btn.setStyleSheet("background-color: white; font-size: 16px; font-weight: bold;")  # 초기 스타일 변경
        self.exit_btn.setFixedHeight(40)
        self.exit_btn.clicked.connect(self.show_exit_confirm)
        
        # 버튼 추가
        button_layout.addWidget(self.parking_btn)
        button_layout.addWidget(self.exit_btn)
        
        # 버튼 영역 추가
        main_layout.addWidget(button_widget)
        
        # 초기 화면 설정 - 시작 화면으로 변경
        self.show_start_screen()
        
        # 초기 데이터로 UI 업데이트
        self.update_parking_info(self.data_manager.get_current_data())
        
        self.show()
    
    def show_start_screen(self):
        """시작 화면 표시"""
        logger.info("시작 화면으로 전환")
        self.stack.setCurrentIndex(0)
        
        # 버튼 상태 업데이트
        self.parking_btn.hide()  # 시작 화면에서는 주차 버튼 숨김
        self.exit_btn.hide()  # 시작 화면에서는 출차 버튼 숨김
        
        self.is_exit_mode = False
    
    def start_parking_flow(self):
        """주차 작업 플로우 시작"""
        # 주차 화면으로 전환 후 주차 확인 다이얼로그 표시
        self.show_parking_screen()
        self.show_parking_confirm()
    
    def start_exit_flow(self):
        """출차 작업 플로우 시작"""
        # 출차 화면으로 전환 후 출차 확인 다이얼로그 표시
        self.show_exit_screen()
        self.show_exit_confirm()
    
    def show_parking_screen(self):
        """주차 화면 표시"""
        logger.info("주차 화면으로 전환")
        self.stack.setCurrentIndex(1)
        
        # 버튼 상태 업데이트
        self.parking_btn.show()
        self.exit_btn.show()
        
        self.parking_btn.setStyleSheet("background-color: #90EE90; font-size: 16px; font-weight: bold;")
        self.exit_btn.setStyleSheet("background-color: white; font-size: 16px; font-weight: bold;")
        self.is_exit_mode = False
        
        # 주차 화면에서는 주차할 차량 정보 표시
        # 차량 번호 정보 표시
        if self.current_license_plate:
            # 인식된 번호판이 있는 경우
            self.update_bottom_info(self.parking_panel, "차량 번호", self.current_license_plate)
            
            # 차량 타입 정보 표시
            car_type_names = {
                "normal": "일반 차량",
                "ev": "전기 차량",
                "disabled": "장애인 차량"
            }
            self.update_car_type_info(self.parking_panel, car_type_names[self.current_car_type])
        else:
            # 인식된 번호판이 없는 경우 기본값 표시
            self.update_bottom_info(self.parking_panel, "차량 번호", "-")
            self.update_car_type_info(self.parking_panel, "일반 차량")
        
        # 차량 위치 정보 표시
        self.update_bottom_info(self.parking_panel, "차량 위치", self.data_manager.current_parking_location)
        
    def show_exit_screen(self):
        """출차 화면 표시"""
        logger.info("출차 화면으로 전환")
        self.stack.setCurrentIndex(2)
        
        # 버튼 상태 업데이트
        self.parking_btn.show()
        self.exit_btn.show()
        
        self.exit_btn.setStyleSheet("background-color: #90EE90; font-size: 16px; font-weight: bold;")
        self.parking_btn.setStyleSheet("background-color: white; font-size: 16px; font-weight: bold;")
        self.is_exit_mode = True
        
        # 출차 화면에서는 주차된 차량 목록 표시
        parked_vehicles = self.data_manager.get_parked_vehicles()
        logger.info(f"출차 화면 전환 - 주차된 차량: {parked_vehicles}")
        
        if parked_vehicles:
            # 첫 번째 주차된 차량 정보 표시
            license_plate = list(parked_vehicles.keys())[0]
            vehicle_info = parked_vehicles.get(license_plate, None)
            
            if vehicle_info:
                if isinstance(vehicle_info, dict):
                    car_type = vehicle_info.get('car_type', 'normal')
                    location = vehicle_info.get('location', '-')
                else:
                    # 이전 버전 호환성을 위한 처리
                    car_type = vehicle_info
                    location = '-'
                
                car_type_names = {
                    "normal": "일반 차량",
                    "ev": "전기 차량",
                    "disabled": "장애인 차량"
                }
                
                car_type_text = car_type_names.get(car_type, "일반 차량")
                
                # 주차 화면과 동일한 방식으로 하단 정보 업데이트
                self.update_bottom_info(self.exit_panel, "차량 번호", license_plate)
                self.update_car_type_info(self.exit_panel, car_type_text)
                self.update_bottom_info(self.exit_panel, "차량 위치", location)
                
                logger.info(f"출차 화면 정보 업데이트: 번호판={license_plate}, 타입={car_type_text}, 위치={location}")
            else:
                # 주차된 차량이 없는 경우
                self.update_bottom_info(self.exit_panel, "차량 번호", "-")
                self.update_car_type_info(self.exit_panel, "일반 차량")
                self.update_bottom_info(self.exit_panel, "차량 위치", "-")
                logger.info("차량 정보가 없습니다")
        else:
            # 주차된 차량이 없는 경우
            self.update_bottom_info(self.exit_panel, "차량 번호", "-")
            self.update_car_type_info(self.exit_panel, "일반 차량")
            self.update_bottom_info(self.exit_panel, "차량 위치", "-")
            logger.info("주차된 차량이 없습니다")
    
    def update_parking_info(self, data):
        """주차장 정보 업데이트"""
        # 주차 패널 업데이트
        self.update_panel_info(self.parking_panel, data)
        
        # 출차 패널 업데이트
        self.update_panel_info(self.exit_panel, data)
    
    def update_panel_info(self, panel, data):
        """패널의 주차 정보 업데이트"""
        # 패널에서 정보 위젯 찾기
        info_widget = panel.findChild(QWidget, "info_widget")
        if not info_widget:
            return
            
        # 각 행 업데이트
        total_row = info_widget.findChild(QWidget, "row_total")
        occupied_row = info_widget.findChild(QWidget, "row_occupied")
        available_row = info_widget.findChild(QWidget, "row_available")
        
        if total_row:
            self.update_row_values(total_row, data["total"])
        
        if occupied_row:
            self.update_row_values(occupied_row, data["occupied"])
        
        if available_row:
            self.update_row_values(available_row, data["available"])
    
    def update_row_values(self, row_widget, values):
        """행의 값 업데이트"""
        # 각 아이콘 라벨 찾기
        normal_label = row_widget.findChild(QWidget, "normal_label")
        ev_label = row_widget.findChild(QWidget, "ev_label")
        disabled_label = row_widget.findChild(QWidget, "disabled_label")
        
        # 값 업데이트
        if normal_label:
            count_label = normal_label.findChild(QLabel, "count_label")
            if count_label:
                count_label.setText(str(values["normal"]))
        
        if ev_label:
            count_label = ev_label.findChild(QLabel, "count_label")
            if count_label:
                count_label.setText(str(values["ev"]))
        
        if disabled_label:
            count_label = disabled_label.findChild(QLabel, "count_label")
            if count_label:
                count_label.setText(str(values["disabled"]))
    
    def show_parking_confirm(self):
        """주차 확인 다이얼로그 표시"""
        # YOLO로 인식된 번호판 정보가 있으면 해당 차량 타입 사용
        if self.current_license_plate:
            # 이미 주차된 차량인지 확인
            if self.current_license_plate in self.data_manager.get_parked_vehicles():
                msg = QMessageBox(self)
                msg.setWindowTitle("주차 불가")
                msg.setText(f"번호판 '{self.current_license_plate}'는 이미 주차되어 있습니다.")
                msg.setIcon(QMessageBox.Warning)
                
                # OK 버튼 스타일 변경
                ok_button = msg.addButton(QMessageBox.Ok)
                ok_button.setText("확인")
                ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                
                msg.exec_()
                # 홈 화면으로 돌아가기
                self.show_start_screen()
                return
                
            car_type = self.current_car_type
            car_type_names = {
                "normal": "일반 차량",
                "ev": "전기 차량",
                "disabled": "장애인 차량"
            }
            
            # 번호판 정보로 주차 확인 메시지
            message = f"번호판: {self.current_license_plate}\n차량 타입: {car_type_names[car_type]}\n\n주차를 시작하시겠습니까?"
            dialog = ConfirmDialog("주차 시작", message, None, self)
            result = dialog.exec_()
            
            if result == QDialog.Accepted:
                # 번호판 정보와 함께 주차 처리
                success, reason = self.data_manager.simulate_parking(car_type, self.current_license_plate)
                
                if success:
                    # 주차 화면으로 전환
                    self.show_parking_screen()
                    
                    # 차량 위치 정보 가져오기
                    location = self.data_manager.get_vehicle_location(self.current_license_plate)
                    # 주차 패널의 하단 정보 업데이트 - 차량 위치 표시
                    self.update_bottom_info(self.parking_panel, "차량 위치", location)
                    # 차량 번호 정보 업데이트
                    self.update_bottom_info(self.parking_panel, "차량 번호", self.current_license_plate)
                    # 차량 타입 정보 업데이트
                    self.update_car_type_info(self.parking_panel, car_type_names[car_type])
                    
                    # 주차 완료 메시지
                    msg = QMessageBox(self)
                    msg.setWindowTitle("주차 완료")
                    msg.setText(f"{car_type_names[car_type]} 주차가 완료되었습니다.\n차량 번호: {self.current_license_plate}\n차량 위치: {location}")
                    msg.setIcon(QMessageBox.Information)
                    
                    # OK 버튼 스타일 변경
                    ok_button = msg.addButton(QMessageBox.Ok)
                    ok_button.setText("확인")
                    ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                    
                    msg.exec_()
                elif reason == "duplicate":
                    msg = QMessageBox(self)
                    msg.setWindowTitle("주차 불가")
                    msg.setText(f"번호판 '{self.current_license_plate}'는 이미 주차되어 있습니다.")
                    msg.setIcon(QMessageBox.Warning)
                    
                    # OK 버튼 스타일 변경
                    ok_button = msg.addButton(QMessageBox.Ok)
                    ok_button.setText("확인")
                    ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                    
                    msg.exec_()
                    # 홈 화면으로 돌아가기
                    self.show_start_screen()
                else:
                    msg = QMessageBox(self)
                    msg.setWindowTitle("주차 불가")
                    msg.setText(f"{car_type_names[car_type]} 주차 공간이 없습니다.")
                    msg.setIcon(QMessageBox.Warning)
                    
                    # OK 버튼 스타일 변경
                    ok_button = msg.addButton(QMessageBox.Ok)
                    ok_button.setText("확인")
                    ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                    
                    msg.exec_()
                    # 홈 화면으로 돌아가기
                    self.show_start_screen()
                
                # 번호판 정보 초기화
                self.current_license_plate = ""
                self.current_car_type = "normal"
            else:
                # 취소 버튼 클릭 시 홈 화면으로 돌아가기
                self.show_start_screen()
        else:
            # 번호판 정보가 없으면 OCR 결과를 기다리라는 메시지 표시
            msg = QMessageBox(self)
            msg.setWindowTitle("차량 인식 대기")
            msg.setText("차량 번호판을 인식 중입니다.\n잠시만 기다려주세요.")
            msg.setIcon(QMessageBox.Information)
            
            # 수동 입력 버튼 추가
            manual_button = msg.addButton(QMessageBox.Ok)
            manual_button.setText("수동 입력")
            manual_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
           
            # 취소 버튼 추가
            ok_button = msg.addButton(QMessageBox.Cancel)
            ok_button.setText("취소")
            ok_button.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")

            result = msg.exec_()
            
            # 수동 입력 버튼 클릭 시
            if msg.clickedButton() == manual_button:
                # 기존 방식으로 차량 타입 선택
                dialog = ConfirmDialog("주차 시작", "주차할 차량 타입을 선택하세요:", ["normal", "ev", "disabled"], self)
                result = dialog.exec_()
                
                if result == QDialog.Accepted:
                    car_type = dialog.get_selected_car_type()
                    
                    # 차량 번호 입력 다이얼로그 표시
                    license_plate = self.show_license_input_dialog()
                    if not license_plate:  # 취소하거나 입력하지 않은 경우
                        # 홈 화면으로 돌아가기
                        self.show_start_screen()
                        return
                    
                    # 입력받은 번호판으로 주차 처리
                    success, reason = self.data_manager.simulate_parking(car_type, license_plate)
                    
                    if success:
                        # 주차 화면으로 전환
                        self.show_parking_screen()
                        
                        # 차량 위치 정보 가져오기
                        location = self.data_manager.get_vehicle_location(license_plate)
                        # 주차 패널의 하단 정보 업데이트 - 차량 위치 표시
                        self.update_bottom_info(self.parking_panel, "차량 위치", location)
                        # 차량 번호 정보 업데이트
                        self.update_bottom_info(self.parking_panel, "차량 번호", license_plate)
                        
                        car_type_names = {
                            "normal": "일반 차량",
                            "ev": "전기 차량",
                            "disabled": "장애인 차량"
                        }
                        # 차량 타입 정보 업데이트
                        self.update_car_type_info(self.parking_panel, car_type_names[car_type])
                        
                        # 주차 완료 메시지
                        msg = QMessageBox(self)
                        msg.setWindowTitle("주차 완료")
                        msg.setText(f"{car_type_names[car_type]} 주차가 완료되었습니다.\n차량 번호: {license_plate}\n차량 위치: {location}")
                        msg.setIcon(QMessageBox.Information)
                        
                        # OK 버튼 스타일 변경
                        ok_button = msg.addButton(QMessageBox.Ok)
                        ok_button.setText("확인")
                        ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                        
                        msg.exec_()
                    elif reason == "duplicate":
                        QMessageBox.warning(self, "주차 불가", f"번호판 '{license_plate}'는 이미 주차되어 있습니다.")
                        # 홈 화면으로 돌아가기
                        self.show_start_screen()
                    else:
                        car_type_names = {
                            "normal": "일반 차량",
                            "ev": "전기 차량",
                            "disabled": "장애인 차량"
                        }
                        QMessageBox.warning(self, "주차 불가", f"{car_type_names[car_type]} 주차 공간이 없습니다.")
                        # 홈 화면으로 돌아가기
                        self.show_start_screen()
                else:
                    # 취소 버튼 클릭 시 홈 화면으로 돌아가기
                    self.show_start_screen()
            else:
                # 확인 버튼 클릭 시 홈 화면으로 돌아가기
                self.show_start_screen()
    
    def show_license_input_dialog(self):
        """차량 번호 입력 다이얼로그 표시"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout
        
        class LicenseInputDialog(QDialog):
            def __init__(self, parent=None):
                super().__init__(parent)
                self.setWindowTitle("차량 번호 입력")
                self.setFixedSize(400, 200)
                self.setStyleSheet("background-color: #3D3B53;")
                
                self.license_number = ""
                
                layout = QVBoxLayout(self)
                layout.setContentsMargins(20, 20, 20, 20)
                
                # 안내 메시지
                info_label = QLabel("차량 번호를 입력하세요 (예: 123가4567):")
                info_label.setStyleSheet("color: white; font-size: 14px;")
                layout.addWidget(info_label)
                
                # 입력 필드
                self.input_field = QLineEdit()
                self.input_field.setStyleSheet("""
                    QLineEdit {
                        background-color: white;
                        color: black;
                        font-size: 18px;
                        padding: 10px;
                        border-radius: 5px;
                    }
                """)
                layout.addWidget(self.input_field)
                
                # 버튼 영역
                btn_layout = QHBoxLayout()
                
                # 확인 버튼
                self.confirm_btn = QPushButton("확인")
                self.confirm_btn.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 10px;")
                self.confirm_btn.clicked.connect(self.accept_input)
                
                # 취소 버튼
                cancel_btn = QPushButton("취소")
                cancel_btn.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 10px;")
                cancel_btn.clicked.connect(self.reject)
                
                btn_layout.addWidget(self.confirm_btn)
                btn_layout.addWidget(cancel_btn)
                
                layout.addLayout(btn_layout)
            
            def accept_input(self):
                """입력된 번호 저장 및 다이얼로그 종료"""
                self.license_number = self.input_field.text().strip()
                
                # 차량 번호 형식 검증 (숫자+한글+숫자)
                # 예: 12가3456, 123나7890
                # 한국 차량 번호판에 사용되는 한글 글자만 허용
                allowed_chars = "가나다라마바사아자하거너더러머버서어저고노도로모보소오조구누두루무부수우주허호"
                pattern = r'^[0-9]{2,3}[' + allowed_chars + r'][0-9]{4}$'
                
                if re.match(pattern, self.license_number):
                    self.accept()
                else:
                    from PyQt5.QtWidgets import QMessageBox
                    msg = QMessageBox()
                    msg.setWindowTitle("입력 오류")
                    msg.setText("올바른 차량 번호 형식이 아닙니다.\n예: 12가3456, 123나7890")
                    msg.setIcon(QMessageBox.Warning)
                    
                    # OK 버튼 스타일 변경
                    ok_button = msg.addButton(QMessageBox.Ok)
                    ok_button.setText("확인")
                    ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                    
                    msg.exec_()
            
            def get_license_number(self):
                """입력된 번호 반환"""
                return self.license_number
        
        # 차량 번호 입력 다이얼로그 표시
        while True:
            input_dialog = LicenseInputDialog(self)
            result = input_dialog.exec_()
            
            if result == QDialog.Accepted:
                license_number = input_dialog.get_license_number()
                
                # 이미 주차된 차량인지 확인
                if license_number in self.data_manager.get_parked_vehicles():
                    QMessageBox.warning(self, "주차 불가", f"번호판 '{license_number}'는 이미 주차되어 있습니다.")
                    continue  # 다시 입력 받기
                
                return license_number
            else:
                # 취소 버튼 클릭 시
                return None
    
    def show_exit_confirm(self):
        """출차 확인 다이얼로그 표시"""
        # 출차 모드 설정
        self.is_exit_mode = True
        
        # 주차된 차량 목록 확인
        parked_vehicles = self.data_manager.get_parked_vehicles()
        
        if not parked_vehicles:
            QMessageBox.warning(self, "출차 불가", "주차된 차량이 없습니다.")
            # 홈 화면으로 돌아가기
            self.show_start_screen()
            return
        
        # 항상 차량 번호 뒷자리 입력 다이얼로그 표시
        self.show_license_search_dialog()
    
    def show_license_search_dialog(self):
        """차량 번호 뒷자리 검색 다이얼로그 표시"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout
        
        class LicenseSearchDialog(QDialog):
            def __init__(self, parent=None):
                super().__init__(parent)
                self.setWindowTitle("차량 번호 검색")
                self.setFixedSize(400, 200)
                self.setStyleSheet("background-color: #3D3B53;")
                
                self.license_number = ""
                
                layout = QVBoxLayout(self)
                layout.setContentsMargins(20, 20, 20, 20)
                
                # 안내 메시지
                info_label = QLabel("출차할 차량의 번호 뒷자리 4자리를 입력하세요:")
                info_label.setStyleSheet("color: white; font-size: 14px;")
                layout.addWidget(info_label)
                
                # 입력 필드
                self.input_field = QLineEdit()
                self.input_field.setStyleSheet("""
                    QLineEdit {
                        background-color: white;
                        color: black;
                        font-size: 18px;
                        padding: 10px;
                        border-radius: 5px;
                    }
                """)
                self.input_field.setMaxLength(4)  # 최대 4자리만 입력 가능
                layout.addWidget(self.input_field)
                
                # 버튼 영역
                btn_layout = QHBoxLayout()
                
                # 검색 버튼
                self.search_btn = QPushButton("검색")
                self.search_btn.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 10px;")
                self.search_btn.clicked.connect(self.accept_input)
                
                # 취소 버튼
                cancel_btn = QPushButton("취소")
                cancel_btn.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 10px;")
                cancel_btn.clicked.connect(self.reject)
                
                btn_layout.addWidget(self.search_btn)
                btn_layout.addWidget(cancel_btn)
                
                layout.addLayout(btn_layout)
            
            def accept_input(self):
                """입력된 번호 저장 및 다이얼로그 종료"""
                self.license_number = self.input_field.text().strip()
                if len(self.license_number) == 4 and self.license_number.isdigit():
                    self.accept()
                else:
                    from PyQt5.QtWidgets import QMessageBox
                    QMessageBox.warning(self, "입력 오류", "4자리 숫자를 입력해주세요.")
            
            def get_license_number(self):
                """입력된 번호 반환"""
                return self.license_number
        
        # 차량 번호 검색 다이얼로그 표시
        search_dialog = LicenseSearchDialog(self)
        result = search_dialog.exec_()
        
        if result == QDialog.Accepted:
            # 입력된 번호로 차량 검색
            license_number = search_dialog.get_license_number()
            self.search_and_exit_vehicle(license_number)
        else:
            # 취소 버튼 클릭 시 홈 화면으로 돌아가기
            self.show_start_screen()
    
    def search_and_exit_vehicle(self, license_number):
        """입력된 번호로 차량을 검색하고 출차 처리"""
        # 주차된 차량 목록 확인
        parked_vehicles = self.data_manager.get_parked_vehicles()
        
        # 뒷자리가 일치하는 차량 찾기
        matching_vehicles = []
        for plate, info in parked_vehicles.items():
            # 번호판의 마지막 4자리가 입력된 번호와 일치하는지 확인
            if plate.endswith(license_number):
                matching_vehicles.append((plate, info))
        
        if not matching_vehicles:
            # 일치하는 차량이 없는 경우
            msg = QMessageBox(self)
            msg.setWindowTitle("검색 결과")
            msg.setText(f"번호 뒷자리 '{license_number}'와 일치하는 차량이 없습니다.")
            
            # 버튼 추가 및 스타일 설정
            retry_button = msg.addButton("다시 검색", QMessageBox.AcceptRole)
            manual_button = msg.addButton("취소", QMessageBox.RejectRole)
            
            # 버튼 스타일 설정
            retry_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
            manual_button.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
            
            msg.exec_()
            
            if msg.clickedButton() == retry_button:
                # 다시 검색
                self.show_license_search_dialog()
            else:
                # 취소 버튼 클릭 시 홈 화면으로 돌아가기
                self.show_start_screen()
            return
        
        # 일치하는 차량이 있는 경우 (여러 대일 수 있음)
        if len(matching_vehicles) == 1:
            # 하나의 차량만 일치하는 경우
            plate, info = matching_vehicles[0]
            self.confirm_exit_vehicle(plate, info)
        else:
            # 여러 차량이 일치하는 경우, 선택 다이얼로그 표시
            self.show_multiple_matches_dialog(matching_vehicles)
    
    def confirm_exit_vehicle(self, license_plate, vehicle_info):
        """출차 확인 다이얼로그 표시"""
        car_type = vehicle_info['car_type']
        location = vehicle_info['location']
        
        car_type_names = {
            "normal": "일반 차량",
            "ev": "전기 차량",
            "disabled": "장애인 차량"
        }
        
        # 출차 확인 메시지
        message = f"차량 번호: {license_plate}\n차량 타입: {car_type_names[car_type]}\n차량 위치: {location}\n\n이 차량을 출차하시겠습니까?"
        dialog = ConfirmDialog("출차 확인", message, None, self)
        result = dialog.exec_()
        
        if result == QDialog.Accepted:
            # 확인 버튼 클릭 시 차량 위치 정보 publish
            if location != '-':
                logger.info(f"출차 확인 - 차량 위치 정보 발행: {location}")
                self.data_manager.publish_location(location)
            
            # 출차 처리
            success, car_type, location = self.data_manager.simulate_leaving(license_plate)
            
            if success:
                # 출차 화면으로 전환
                self.show_exit_screen()
                
                # 출차 완료 메시지
                msg = QMessageBox(self)
                msg.setWindowTitle("출차 완료")
                msg.setText(f"{car_type_names[car_type]} 출차가 완료되었습니다.\n차량 번호: {license_plate}")
                msg.setIcon(QMessageBox.Information)
                
                # OK 버튼 스타일 변경
                ok_button = msg.addButton(QMessageBox.Ok)
                ok_button.setText("확인")
                ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                
                msg.exec_()
                
                # 현재 선택된 차량 정보 업데이트
                self.current_license_plate = ""
                self.current_car_type = "normal"
            else:
                msg = QMessageBox(self)
                msg.setWindowTitle("출차 불가")
                msg.setText("주차된 차량을 찾을 수 없습니다.")
                msg.setIcon(QMessageBox.Warning)
                
                # OK 버튼 스타일 변경
                ok_button = msg.addButton(QMessageBox.Ok)
                ok_button.setText("확인")
                ok_button.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 5px 15px; min-width: 80px;")
                
                msg.exec_()
                # 홈 화면으로 돌아가기
                self.show_start_screen()
        else:
            # 취소 시 홈 화면으로 돌아가기
            self.show_start_screen()
    
    def show_multiple_matches_dialog(self, matching_vehicles):
        """여러 차량이 일치하는 경우 선택 다이얼로그 표시"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton, QHBoxLayout
        
        class VehicleSelectDialog(QDialog):
            def __init__(self, vehicles, parent=None):
                super().__init__(parent)
                self.setWindowTitle("차량 선택")
                self.setFixedSize(500, 400)
                self.setStyleSheet("background-color: #3D3B53;")
                
                self.selected_vehicle = None
                self.selected_info = None
                
                layout = QVBoxLayout(self)
                layout.setContentsMargins(20, 20, 20, 20)
                
                # 안내 메시지
                info_label = QLabel("여러 차량이 검색되었습니다. 출차할 차량을 선택하세요:")
                info_label.setStyleSheet("color: white; font-size: 16px;")
                layout.addWidget(info_label)
                
                # 차량 목록
                self.vehicle_list = QListWidget()
                self.vehicle_list.setStyleSheet("""
                    QListWidget {
                        background-color: #534F77;
                        color: white;
                        font-size: 14px;
                        border-radius: 10px;
                        padding: 10px;
                    }
                    QListWidget::item {
                        padding: 10px;
                        border-bottom: 1px solid #6C6C6C;
                    }
                    QListWidget::item:selected {
                        background-color: #90EE90;
                        color: black;
                    }
                """)
                
                # 차량 타입 이름 매핑
                car_type_names = {
                    "normal": "일반 차량",
                    "ev": "전기 차량",
                    "disabled": "장애인 차량"
                }
                
                # 차량 목록 추가
                for license_plate, info in vehicles:
                    car_type = info.get('car_type', 'normal')
                    location = info.get('location', '-')
                    car_type_name = car_type_names.get(car_type, "일반 차량")
                    
                    item_text = f"번호판: {license_plate} | 차량 타입: {car_type_name} | 위치: {location}"
                    item = QListWidgetItem(item_text)
                    item.setData(Qt.UserRole, (license_plate, info))  # 번호판 정보와 차량 정보 저장
                    self.vehicle_list.addItem(item)
                
                layout.addWidget(self.vehicle_list)
                
                # 버튼 영역
                btn_layout = QHBoxLayout()
                
                # 확인 버튼
                self.ok_btn = QPushButton("선택")
                self.ok_btn.setStyleSheet("background-color: #90EE90; color: black; font-size: 14px; font-weight: bold; padding: 10px;")
                self.ok_btn.clicked.connect(self.accept_selection)
                self.ok_btn.setEnabled(False)  # 초기에는 비활성화
                
                # 취소 버튼
                cancel_btn = QPushButton("취소")
                cancel_btn.setStyleSheet("background-color: #FF6B6B; color: white; font-size: 14px; font-weight: bold; padding: 10px;")
                cancel_btn.clicked.connect(self.reject)
                
                btn_layout.addWidget(self.ok_btn)
                btn_layout.addWidget(cancel_btn)
                
                layout.addLayout(btn_layout)
                
                # 항목 선택 시 버튼 활성화
                self.vehicle_list.itemSelectionChanged.connect(self.on_selection_changed)
            
            def on_selection_changed(self):
                # 항목이 선택되면 출차 버튼 활성화
                self.ok_btn.setEnabled(len(self.vehicle_list.selectedItems()) > 0)
            
            def accept_selection(self):
                # 선택된 항목의 번호판 정보 저장
                selected_items = self.vehicle_list.selectedItems()
                if selected_items:
                    self.selected_vehicle, self.selected_info = selected_items[0].data(Qt.UserRole)
                self.accept()
            
            def get_selected_vehicle(self):
                return self.selected_vehicle, self.selected_info
        
        # 차량 선택 다이얼로그 표시
        dialog = VehicleSelectDialog(matching_vehicles, self)
        result = dialog.exec_()
        
        if result == QDialog.Accepted:
            # 선택된 차량 정보 가져오기
            license_plate, vehicle_info = dialog.get_selected_vehicle()
            # 출차 확인 다이얼로그 표시
            self.confirm_exit_vehicle(license_plate, vehicle_info)
        else:
            # 취소 시 홈 화면으로 돌아가기
            self.show_start_screen()
    
    def update_car_type_info(self, panel, car_type_text):
        """차량 타입 정보 업데이트"""
        # 패널에서 차량 타입 정보 위젯 찾기
        car_icon = panel.findChild(QLabel, "car_icon")
        if car_icon:
            # 차량 타입에 따라 아이콘 변경
            if "전기" in car_type_text:
                car_icon.setText("⚡🚙")
            elif "장애인" in car_type_text:
                car_icon.setText("♿")
            else:
                car_icon.setText("🚗")
            
            # 차량 타입 텍스트 표시
            info_label = panel.findChild(QLabel, "car_type_label")
            if info_label:
                info_label.setText(car_type_text)
    
    def create_panel(self, title, image_path, location_text, location_value):
        panel = QWidget()
        panel.setStyleSheet("background-color: #2D2B43; border-radius: 20px;")
        
        layout = QVBoxLayout(panel)
        layout.setSpacing(12)  
        
        # 타이틀
        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: white; font-size: 22px; font-weight: bold;")
        title_label.setFixedHeight(40)
        layout.addWidget(title_label)
        
        # 주차 구역 정보
        info_widget = QWidget()
        info_widget.setObjectName("info_widget")
        info_widget.setStyleSheet("background-color: #3D3B53; border-radius: 10px;")
        info_layout = QVBoxLayout(info_widget)
        info_layout.setSpacing(5) 
        info_layout.setContentsMargins(10, 5, 10, 5)
        
        # 주차 구역 레이블과 초기값
        parking_info = [
            ("주차 구역", "total", "row_total"),
            ("사용중", "occupied", "row_occupied"),
            ("주차 가능", "available", "row_available")
        ]
        
        # 현재 데이터 가져오기
        current_data = self.data_manager.get_current_data()
        
        for info in parking_info:
            row_widget = QWidget()
            row_widget.setObjectName(info[2])  # 행 식별자 설정
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(10, 3, 10, 3)
            
            # 레이블
            label = QLabel(info[0])
            label.setStyleSheet("color: white; font-size: 20px;")
            label.setMinimumWidth(100)  
            row_layout.addWidget(label)
            
            # 데이터 타입에 따른 값 가져오기
            data_type = info[1]
            values = current_data.get(data_type, {"normal": 0, "ev": 0, "disabled": 0})
            
            # 아이콘과 숫자들
            car_label = self.create_icon_label("🚗", values["normal"], "normal_label")
            ev_label = self.create_icon_label("⚡🚙", values["ev"], "ev_label")
            disabled_label = self.create_icon_label("♿", values["disabled"], "disabled_label")
            
            # 아이콘 간 간격 추가
            row_layout.addSpacing(15)
            row_layout.addWidget(car_label)
            row_layout.addSpacing(20)  
            row_layout.addWidget(ev_label)
            row_layout.addSpacing(20)  
            row_layout.addWidget(disabled_label)
            row_layout.addStretch()
            
            info_layout.addWidget(row_widget)
        
        layout.addWidget(info_widget)
        
        # 이미지 영역
        image_widget = QWidget()
        image_widget.setStyleSheet("background-color: #534F77; border-radius: 15px;")
        image_widget.setMinimumHeight(320)
        image_layout = QVBoxLayout(image_widget)
        image_layout.setContentsMargins(10, 10, 10, 10)
        
        # 이미지 라벨 생성
        image_label = QLabel()
        image_label.setObjectName("image_label")  # 이미지 라벨에 ID 부여
        image_label.setAlignment(Qt.AlignCenter)
        
        # 기본 이미지 설정
        try:
            pixmap = QPixmap(image_path)
            image_label.setPixmap(pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except:
            # 이미지가 없는 경우 대체 텍스트
            image_label.setText("이미지 없음")
            image_label.setStyleSheet("color: white; font-size: 18px;")
        
        image_layout.addWidget(image_label)
        layout.addWidget(image_widget)
        
        # 하단 정보 표시 - 그리드 레이아웃으로 변경하여 더 많은 정보 표시
        bottom_widget = QWidget()
        bottom_widget.setObjectName("bottom_info")  # 하단 정보 위젯 ID 추가
        bottom_widget.setStyleSheet("background-color: #534F77; border-radius: 10px;")
        bottom_widget.setFixedHeight(150)  # 높이 증가
        
        # 그리드 레이아웃으로 변경하여 정확한 위치 지정
        bottom_layout = QGridLayout(bottom_widget)
        bottom_layout.setContentsMargins(15, 10, 15, 10)  # 여백 증가
        bottom_layout.setVerticalSpacing(15)  # 수직 간격 증가
        
        # 차량 아이콘 - 항상 왼쪽에 배치
        car_icon = QLabel("🚗")
        car_icon.setObjectName("car_icon")
        car_icon.setStyleSheet("color: yellow; font-size: 28px;")  # 폰트 크기 증가
        car_icon.setFixedWidth(40)
        car_icon.setAlignment(Qt.AlignLeft)
        bottom_layout.addWidget(car_icon, 0, 0, 1, 1)
        
        # 차량 타입 표시 - 첫 번째 행
        car_type_label = QLabel("일반 차량")
        car_type_label.setObjectName("car_type_label")
        car_type_label.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")  # 폰트 크기 증가
        car_type_label.setAlignment(Qt.AlignLeft)
        car_type_label.setContentsMargins(10, 0, 0, 0)
        car_type_label.setMinimumWidth(200)  # 최소 너비 설정하여 글씨 잘림 방지
        bottom_layout.addWidget(car_type_label, 0, 1, 1, 2)
        
        # 차량 번호 정보 - 두 번째 행
        loc_label = QLabel("차량 번호")
        loc_label.setObjectName("info_label")  # 레이블 ID 추가
        loc_label.setStyleSheet("color: white; font-size: 18px;")  # 폰트 크기 증가
        loc_label.setAlignment(Qt.AlignRight)
        loc_label.setFixedWidth(120)  
        bottom_layout.addWidget(loc_label, 1, 0, 1, 1)
        
        # 차량 번호 값 - 두 번째 행
        value_label = QLabel("-")
        value_label.setObjectName("info_value")  # 값 ID 추가
        value_label.setStyleSheet("color: red; font-size: 18px; font-weight: bold;")  # 폰트 크기 증가
        value_label.setAlignment(Qt.AlignLeft)  
        value_label.setContentsMargins(10, 0, 0, 0)
        value_label.setMinimumWidth(200)  # 최소 너비 설정하여 글씨 잘림 방지
        bottom_layout.addWidget(value_label, 1, 1, 1, 2)
        
        # 차량 위치 정보 - 세 번째 행
        loc_pos_label = QLabel("차량 위치")
        loc_pos_label.setObjectName("pos_label")  # 레이블 ID 추가
        loc_pos_label.setStyleSheet("color: white; font-size: 18px;")  # 폰트 크기 증가
        loc_pos_label.setAlignment(Qt.AlignRight)
        loc_pos_label.setFixedWidth(120)
        bottom_layout.addWidget(loc_pos_label, 2, 0, 1, 1)
        
        # 위치 값 - 세 번째 행
        pos_value_label = QLabel("-")
        pos_value_label.setObjectName("pos_value")  # 값 ID 추가
        pos_value_label.setStyleSheet("color: #90EE90; font-size: 18px; font-weight: bold;")  # 폰트 크기 증가
        pos_value_label.setAlignment(Qt.AlignLeft)
        pos_value_label.setContentsMargins(10, 0, 0, 0)
        pos_value_label.setMinimumWidth(200)  # 최소 너비 설정하여 글씨 잘림 방지
        bottom_layout.addWidget(pos_value_label, 2, 1, 1, 2)
        
        # 그리드 레이아웃 비율 설정
        bottom_layout.setColumnStretch(0, 1)  
        bottom_layout.setColumnStretch(1, 2)  
        bottom_layout.setColumnStretch(2, 3)  
        
        layout.addWidget(bottom_widget)
        
        return panel
        
    def create_icon_label(self, icon, count, object_name=None):
        widget = QWidget()
        if object_name:
            widget.setObjectName(object_name)
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(3, 0, 3, 0)  
        layout.setSpacing(5)
        
        icon_label = QLabel(icon)
        icon_label.setStyleSheet("font-size: 22px;")  
        
        count_label = QLabel(str(count))
        count_label.setObjectName("count_label")  # 값 업데이트를 위한 식별자
        count_label.setStyleSheet("color: white; font-size: 20px;")  
        
        layout.addWidget(icon_label)
        layout.addWidget(count_label)
        
        return widget

    def update_bottom_info(self, panel, label_text, value_text):
        """
        패널 하단 정보 업데이트
        
        Args:
            panel: 업데이트할 패널 위젯
            label_text: 레이블 텍스트
            value_text: 값 텍스트
        """
        # 하단 정보 위젯 찾기
        bottom_widget = panel.findChild(QWidget, "bottom_info")
        if not bottom_widget:
            return
        
        # 레이블에 따라 다른 위젯 업데이트
        if label_text == "차량 번호":
            # 차량 번호 정보 업데이트
            loc_label = bottom_widget.findChild(QLabel, "info_label")
            value_label = bottom_widget.findChild(QLabel, "info_value")
            
            if loc_label and value_label:
                loc_label.setText(label_text)
                value_label.setText(value_text)
        elif label_text == "차량 타입":
            # 차량 타입 정보 업데이트
            car_type_label = bottom_widget.findChild(QLabel, "car_type_label")
            
            if car_type_label:
                car_type_label.setText(value_text)
        elif label_text == "차량 위치":
            # 차량 위치 정보 업데이트
            pos_label = bottom_widget.findChild(QLabel, "pos_label")
            pos_value = bottom_widget.findChild(QLabel, "pos_value")
            
            if pos_label and pos_value:
                pos_label.setText(label_text)
                pos_value.setText(value_text)

    def on_camera_frame_updated(self, q_img):
        """
        카메라 프레임 업데이트 처리
        
        Args:
            q_img (QImage): 카메라에서 캡처한 프레임을 변환한 QImage 객체
        """
        # 현재 화면에 따라 다른 패널에 영상 표시
        if self.stack.currentIndex() == 1:  # 주차 화면
            # 이미지 라벨 찾기
            image_label = self.parking_panel.findChild(QLabel, "image_label")
            
            if image_label:
                # QImage를 QPixmap으로 변환하여 표시
                pixmap = QPixmap.fromImage(q_img)
                image_label.setPixmap(pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
        elif self.stack.currentIndex() == 2:  # 출차 화면
            # 이미지 라벨 찾기
            image_label = self.exit_panel.findChild(QLabel, "image_label")
            
            if image_label:
                # QImage를 QPixmap으로 변환하여 표시
                pixmap = QPixmap.fromImage(q_img)
                image_label.setPixmap(pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def on_ocr_result_updated(self, license_plate, car_type):
        """OCR 결과 업데이트 처리"""
        logger.info(f"OCR 결과 업데이트: 번호판={license_plate}, 타입={car_type}")
        
        # 현재 차량 정보 업데이트
        self.current_license_plate = license_plate
        self.current_car_type = car_type
        
        # 현재 화면에 따라 정보 업데이트
        if self.stack.currentIndex() == 1:  # 주차 화면
            # 차량 번호 정보 업데이트
            self.update_bottom_info(self.parking_panel, "차량 번호", license_plate)
            
            # 차량 타입 정보 업데이트
            car_type_names = {
                "normal": "일반 차량",
                "ev": "전기 차량",
                "disabled": "장애인 차량"
            }
            self.update_car_type_info(self.parking_panel, car_type_names[car_type])
        
        elif self.stack.currentIndex() == 2:  # 출차 화면
            # 차량 번호 정보 업데이트
            self.update_bottom_info(self.exit_panel, "차량 번호", license_plate)
            
            # 차량 타입 정보 업데이트
            car_type_names = {
                "normal": "일반 차량",
                "ev": "전기 차량",
                "disabled": "장애인 차량"
            }
            self.update_car_type_info(self.exit_panel, car_type_names[car_type])

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 일관된 스타일 적용
    
    # 다크 테마 설정
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    app.setPalette(palette)
    
    ex = ParkingManagementGUI()
    sys.exit(app.exec_()) 

if __name__ == '__main__':
    main() 