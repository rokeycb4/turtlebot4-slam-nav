from influxdb_client_3 import InfluxDBClient3
import pandas as pd
from datetime import datetime
import logging
import os
from dotenv import load_dotenv

# .env 파일에서 환경 변수 로드
load_dotenv()

# 환경 변수에서 설정 값 가져오기
org = os.getenv("INFLUXDB_ORG", "project")
token = os.getenv("INFLUXDB_TOKEN", "")
host = os.getenv("INFLUXDB_HOST", "us-east-1-1.aws.cloud2.influxdata.com")
database = os.getenv("INFLUXDB_DATABASE", "parking")

# 로거 설정
logger = logging.getLogger('db_manager')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class DBManager:
    """InfluxDB 클라우드 데이터베이스 관리자 클래스"""
    
    def __init__(self):
        """데이터베이스 관리자 초기화"""
        # InfluxDB 클라이언트 초기화
        self.client = InfluxDBClient3(
            host=host,
            token=token,
            database=database
        )
        logger.info("InfluxDB 클라이언트 초기화 완료")
    
    def fetch_current_parked_vehicles(self):
        """현재 주차된 차량 정보 조회"""
        try:
            logger.info('현재 주차된 차량 정보 조회 중...')
            
            # 최적화된 쿼리: 필요한 필드만 조회하고 최근 30일 데이터만 조회
            latest_status_query = """
            WITH latest_status AS (
                SELECT license_plate, MAX(time) as latest_time
                FROM parking
                WHERE time >= now() - INTERVAL '30 days'
                GROUP BY license_plate
            )
            SELECT p.license_plate, p.car_type, p.location, p.status, p.time
            FROM parking p
            JOIN latest_status ls ON p.license_plate = ls.license_plate AND p.time = ls.latest_time
            WHERE p.status = 'parked'
            ORDER BY p.time DESC
            """
            
            parked_table = self.client.query(latest_status_query)
            parked_df = parked_table.to_pandas() if parked_table is not None else pd.DataFrame()
            
            if not parked_df.empty:
                # 중복 제거 (같은 위치에 여러 차량이 있는 경우 최신 데이터만 유지)
                parked_df = parked_df.sort_values('time', ascending=False).drop_duplicates(subset=['location'])
                logger.info(f"현재 주차된 차량: {len(parked_df)}대")
            else:
                logger.info("현재 주차된 차량이 없습니다.")
            
            return parked_df
        
        except Exception as e:
            logger.error(f"주차 차량 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return pd.DataFrame()
    
    def fetch_recent_exit_records(self, days=7, limit=20):
        """최근 출차 기록 조회"""
        try:
            logger.info(f'최근 {days}일 출차 기록 조회 중...')
            
            # 최적화된 쿼리: 필요한 필드만 조회하고 결과 개수 제한
            exit_query = f"""
            SELECT license_plate, car_type, location, status, time
            FROM parking
            WHERE time >= now() - INTERVAL '{days} days'
            AND status = 'exit'
            ORDER BY time DESC
            LIMIT {limit}
            """
            
            exit_table = self.client.query(exit_query)
            exit_df = exit_table.to_pandas() if exit_table is not None else pd.DataFrame()
            
            if not exit_df.empty:
                logger.info(f"최근 출차 기록: {len(exit_df)}건")
            else:
                logger.info("최근 출차 기록이 없습니다.")
            
            return exit_df
        
        except Exception as e:
            logger.error(f"출차 기록 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return pd.DataFrame()
    
    def fetch_all_parking_data(self, days=30, limit=100):
        """모든 주차 데이터 조회 (주차 + 출차)"""
        try:
            logger.info(f'모든 주차 데이터 조회 중 (최근 {days}일)...')
            
            # 최적화된 쿼리: 직접 InfluxDB에서 모든 데이터를 한 번에 조회
            query = f"""
            SELECT license_plate, car_type, location, status, time
            FROM parking
            WHERE time >= now() - INTERVAL '{days} days'
            ORDER BY time DESC
            LIMIT {limit}
            """
            
            table = self.client.query(query)
            combined_df = table.to_pandas() if table is not None else pd.DataFrame()
            
            if combined_df.empty:
                logger.info("조회된 데이터가 없습니다.")
                return None
            
            logger.info(f"총 {len(combined_df)}개의 데이터를 조회했습니다.")
            return combined_df
        
        except Exception as e:
            logger.error(f"데이터 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def insert_parking_record(self, license_plate, car_type, location, status="parked"):
        """주차/출차 기록 추가"""
        try:
            logger.info(f"주차/출차 기록 추가: {license_plate}, {car_type}, {location}, {status}")
            
            # 현재 시간 (RFC3339 형식)
            timestamp = int(datetime.now().timestamp())
            
            # Line Protocol 형식으로 데이터 생성
            # measurement,tag1=value1,tag2=value2 field1=value1,field2=value2 timestamp
            line_protocol = f"parking,license_plate={license_plate},car_type={car_type},location={location} status=\"{status}\" {timestamp}"
            
            # InfluxDB에 데이터 쓰기
            self.client.write([line_protocol], write_precision='s')
            
            logger.info(f"데이터 추가 완료: {line_protocol}")
            return True
            
        except Exception as e:
            logger.error(f"데이터 추가 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def park_vehicle(self, license_plate, car_type, location):
        """차량 주차 처리"""
        # 차량 타입과 위치의 일치 여부 확인
        valid_location = False
        
        # 일반 차량은 A-1, A-2에만 주차 가능
        if car_type == "normal" and location in ["A-1", "A-2"]:
            valid_location = True
        # 전기 차량은 B-1, B-2에만 주차 가능
        elif car_type == "ev" and location in ["B-1", "B-2"]:
            valid_location = True
        # 장애인 차량은 C-1, C-2에만 주차 가능
        elif car_type == "disabled" and location in ["C-1", "C-2"]:
            valid_location = True
        
        # 위치가 유효하지 않은 경우
        if not valid_location:
            logger.warning(f"유효하지 않은 주차 위치: {car_type} 차량은 {location}에 주차할 수 없습니다.")
            return False
            
        return self.insert_parking_record(license_plate, car_type, location, "parked")
    
    def exit_vehicle(self, license_plate, car_type, location):
        """차량 출차 처리"""
        return self.insert_parking_record(license_plate, car_type, location, "exit")
    
    def get_vehicle_info(self, license_plate):
        """번호판으로 차량 정보 조회"""
        try:
            logger.info(f"차량 정보 조회: {license_plate}")
            
            # 해당 번호판의 가장 최근 주차 상태 조회 (status = 'parked'인 경우만)
            query = f"""
            SELECT license_plate, car_type, location, status, time
            FROM parking
            WHERE license_plate = '{license_plate}'
            AND status = 'parked'
            ORDER BY time DESC
            LIMIT 1
            """
            
            table = self.client.query(query)
            df = table.to_pandas() if table is not None else pd.DataFrame()
            
            if df.empty:
                logger.info(f"번호판 {license_plate}에 해당하는 주차된 차량 정보가 없습니다.")
                return None
            
            # 차량 정보 반환
            vehicle_info = {
                'license_plate': df.iloc[0]['license_plate'],
                'car_type': df.iloc[0]['car_type'],
                'location': df.iloc[0]['location'],
                'status': df.iloc[0]['status'],
                'time': df.iloc[0]['time']
            }
            
            logger.info(f"조회된 차량 정보: {vehicle_info}")
            return vehicle_info
            
        except Exception as e:
            logger.error(f"차량 정보 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def get_vehicle_by_location(self, location):
        """위치로 차량 정보 조회"""
        try:
            logger.info(f"위치 {location}의 차량 정보 조회")
            
            # 해당 위치의 가장 최근 주차 상태 조회
            query = f"""
            SELECT license_plate, car_type, location, status, time
            FROM parking
            WHERE location = '{location}'
            AND status = 'parked'
            ORDER BY time DESC
            LIMIT 1
            """
            
            table = self.client.query(query)
            df = table.to_pandas() if table is not None else pd.DataFrame()
            
            if df.empty:
                logger.info(f"위치 {location}에 주차된 차량이 없습니다.")
                return None
            
            # 차량 정보 반환
            vehicle_info = {
                'license_plate': df.iloc[0]['license_plate'],
                'car_type': df.iloc[0]['car_type'],
                'location': df.iloc[0]['location'],
                'status': df.iloc[0]['status'],
                'time': df.iloc[0]['time']
            }
            
            logger.info(f"조회된 차량 정보: {vehicle_info}")
            return vehicle_info
            
        except Exception as e:
            logger.error(f"차량 정보 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def find_vehicles_by_partial_plate(self, partial_plate):
        """번호판 뒷자리로 차량 검색"""
        try:
            logger.info(f"번호판 뒷자리 {partial_plate}로 차량 검색")
            
            # 현재 주차된 차량 중 번호판 뒷자리가 일치하는 차량 검색
            query = f"""
            SELECT license_plate, car_type, location, status, time
            FROM parking
            WHERE status = 'parked'
            AND license_plate LIKE '%{partial_plate}'
            ORDER BY time DESC
            """
            
            table = self.client.query(query)
            df = table.to_pandas() if table is not None else pd.DataFrame()
            
            if df.empty:
                logger.info(f"번호판 뒷자리 {partial_plate}와 일치하는 주차된 차량이 없습니다.")
                return []
            
            # 중복 제거 (같은 번호판이 여러 번 나올 경우 최신 데이터만 유지)
            df = df.sort_values('time', ascending=False).drop_duplicates(subset=['license_plate'])
            
            # 차량 정보 목록 반환
            vehicles = []
            for _, row in df.iterrows():
                vehicle_info = {
                    'license_plate': row['license_plate'],
                    'car_type': row['car_type'],
                    'location': row['location'],
                    'status': row['status'],
                    'time': row['time']
                }
                vehicles.append(vehicle_info)
            
            logger.info(f"검색된 차량 수: {len(vehicles)}")
            return vehicles
            
        except Exception as e:
            logger.error(f"번호판 검색 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def get_parking_statistics(self, parked_df=None):
        """주차장 통계 정보 조회"""
        try:
            # 차량 타입별 통계 기본값
            statistics = {
                "total": {"normal": 2, "ev": 2, "disabled": 2},  # 기본값
                "occupied": {"normal": 0, "ev": 0, "disabled": 0},
                "available": {"normal": 2, "ev": 2, "disabled": 2}
            }
            
            # 주차된 차량 데이터가 전달되지 않은 경우 조회
            if parked_df is None:
                parked_df = self.fetch_current_parked_vehicles()
            
            # 주차된 차량이 있는 경우 통계 계산
            if not parked_df.empty:
                # 차량 타입별 주차 수 계산
                for car_type in ["normal", "ev", "disabled"]:
                    count = len(parked_df[parked_df['car_type'] == car_type])
                    statistics["occupied"][car_type] = count
                    # 가용 공간 계산 (음수가 되지 않도록)
                    available = max(0, statistics["total"][car_type] - count)
                    statistics["available"][car_type] = available
            
            logger.info(f"주차장 통계: {statistics}")
            return statistics
            
        except Exception as e:
            logger.error(f"주차장 통계 조회 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
            # 기본값 반환
            return {
                "total": {"normal": 2, "ev": 2, "disabled": 2},
                "occupied": {"normal": 0, "ev": 0, "disabled": 0},
                "available": {"normal": 2, "ev": 2, "disabled": 2}
            }
    

# 단독 실행 시 테스트 코드
if __name__ == "__main__":
    import sys
    
    db_manager = DBManager()
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "--stats":
            # 주차장 통계 조회
            stats = db_manager.get_parking_statistics()
            print("주차장 통계:")
            print(f"- 총 주차 공간: 일반 {stats['total']['normal']}대, 전기 {stats['total']['ev']}대, 장애인 {stats['total']['disabled']}대")
            print(f"- 사용 중: 일반 {stats['occupied']['normal']}대, 전기 {stats['occupied']['ev']}대, 장애인 {stats['occupied']['disabled']}대")
            print(f"- 가용 공간: 일반 {stats['available']['normal']}대, 전기 {stats['available']['ev']}대, 장애인 {stats['available']['disabled']}대")
        
        elif command == "--parked":
            # 현재 주차된 차량 조회
            parked_df = db_manager.fetch_current_parked_vehicles()
            if not parked_df.empty:
                print("\n현재 주차된 차량:")
                print("-" * 60)
                print(f"{'번호판':<12} | {'차량 타입':<10} | {'위치':<8} | {'주차 시간':<20}")
                print("-" * 60)
                
                for _, row in parked_df.iterrows():
                    time_str = pd.to_datetime(row['time']).strftime('%Y-%m-%d %H:%M:%S')
                    car_type_name = {'normal': '일반', 'ev': '전기', 'disabled': '장애인'}.get(row['car_type'], row['car_type'])
                    print(f"{row['license_plate']:<12} | {car_type_name:<10} | {row['location']:<8} | {time_str}")
            else:
                print("현재 주차된 차량이 없습니다.")
        
        elif command == "--exits":
            # 최근 출차 기록 조회
            exit_df = db_manager.fetch_recent_exit_records()
            if not exit_df.empty:
                print("\n최근 출차 기록:")
                print("-" * 60)
                print(f"{'번호판':<12} | {'차량 타입':<10} | {'위치':<8} | {'출차 시간':<20}")
                print("-" * 60)
                
                for _, row in exit_df.iterrows():
                    time_str = pd.to_datetime(row['time']).strftime('%Y-%m-%d %H:%M:%S')
                    car_type_name = {'normal': '일반', 'ev': '전기', 'disabled': '장애인'}.get(row['car_type'], row['car_type'])
                    print(f"{row['license_plate']:<12} | {car_type_name:<10} | {row['location']:<8} | {time_str}")
            else:
                print("최근 출차 기록이 없습니다.")
        
        elif command == "--park":
            # 차량 주차 처리
            if len(sys.argv) >= 5:
                license_plate = sys.argv[2]
                car_type = sys.argv[3]
                location = sys.argv[4]
                
                success = db_manager.park_vehicle(license_plate, car_type, location)
                if success:
                    print(f"차량 주차 완료: {license_plate}, {car_type}, {location}")
                else:
                    print("차량 주차 처리 중 오류가 발생했습니다.")
            else:
                print("사용법: python db_manager.py --park <번호판> <차량타입> <위치>")
        
        elif command == "--exit":
            # 차량 출차 처리
            if len(sys.argv) >= 3:
                license_plate = sys.argv[2]
                
                # 차량 정보 조회
                vehicle_info = db_manager.get_vehicle_info(license_plate)
                if vehicle_info and vehicle_info['status'] == 'parked':
                    car_type = vehicle_info['car_type']
                    location = vehicle_info['location']
                    
                    success = db_manager.exit_vehicle(license_plate, car_type, location)
                    if success:
                        print(f"차량 출차 완료: {license_plate}, {car_type}, {location}")
                    else:
                        print("차량 출차 처리 중 오류가 발생했습니다.")
                else:
                    print(f"주차된 차량 {license_plate}를 찾을 수 없습니다.")
            else:
                print("사용법: python db_manager.py --exit <번호판>")
        
        elif command == "--search":
            # 번호판으로 차량 검색
            if len(sys.argv) >= 3:
                partial_plate = sys.argv[2]
                vehicles = db_manager.find_vehicles_by_partial_plate(partial_plate)
                
                if vehicles:
                    print(f"\n번호판 '{partial_plate}'로 검색된 차량:")
                    print("-" * 60)
                    print(f"{'번호판':<12} | {'차량 타입':<10} | {'위치':<8}")
                    print("-" * 60)
                    
                    for vehicle in vehicles:
                        car_type_name = {'normal': '일반', 'ev': '전기', 'disabled': '장애인'}.get(vehicle['car_type'], vehicle['car_type'])
                        print(f"{vehicle['license_plate']:<12} | {car_type_name:<10} | {vehicle['location']:<8}")
                else:
                    print(f"번호판 '{partial_plate}'로 검색된 차량이 없습니다.")
            else:
                print("사용법: python db_manager.py --search <번호판_뒷자리>")
        
        
        else:
            print("알 수 없는 명령어입니다.")
            print("사용 가능한 명령어: --stats, --parked, --exits, --park, --exit, --search")
    
    else:
        # 기본 실행 - 모든 주차 데이터 조회
        data = db_manager.fetch_all_parking_data()
        if data is not None and not data.empty:
            print("\n조회된 주차 데이터:")
            print(data.head(10))  # 처음 10개 데이터만 출력
        else:
            print("조회된 데이터가 없습니다.") 