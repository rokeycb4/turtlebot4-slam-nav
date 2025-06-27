from yolosplitter import YoloSplitter
import shutil
import os

# 현재 스크립트 실행 위치
BASE_DIR = os.getcwd()

# 원본 데이터 폴더
IMAGE_DIR = "/home/mi/rokey_ws/images"

# 1. YoloSplitter 설정
ys = YoloSplitter(imgFormat=['.jpg', '.jpeg', '.png'], labelFormat=['.txt'])

# 2. 분할 실행
df = ys.from_mixed_dir(input_dir=IMAGE_DIR, ratio=(0.7, 0.2, 0.1), return_df=True)

# 3. 결과 확인
ys.info()
print(ys.get_dataframe())

# 4. 분할 저장 경로 설정
output_folder_name = os.path.basename(IMAGE_DIR) + "_split"
output_folder = os.path.join(BASE_DIR, output_folder_name)

# 기존 분할 폴더가 있으면 삭제
if os.path.exists(output_folder):
    shutil.rmtree(output_folder)
    print(f"기존 폴더 삭제: {output_folder}")

# 5. 새로 저장
ys.save_split(output_dir=output_folder)

# 6. 압축
zip_filename = output_folder + ".zip"

if os.path.exists(zip_filename):
    os.remove(zip_filename)

shutil.make_archive(base_name=output_folder, format='zip', root_dir=output_folder)
print(f"압축 완료: {zip_filename}")
