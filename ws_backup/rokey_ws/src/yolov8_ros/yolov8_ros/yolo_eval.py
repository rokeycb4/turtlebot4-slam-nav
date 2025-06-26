from ultralytics import YOLO
import os

class YoloEval:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def eval(self, image_path_list, conf=0.25, img_save=False):
        results = self.model.predict(source=image_path_list, conf=conf)
        for i, result in enumerate(results):
            if img_save:
                result.save(filename=f'result_{i}.png')

if __name__ == "__main__":
    image_dir = "/home/fred/rokey_ws/images"
    image_list = [
        os.path.join(image_dir, f)
        for f in os.listdir(image_dir)
        if f.endswith((".png", ".jpg", ".jpeg"))
    ]

    yolo_eval = YoloEval("/home/fred/rokey_ws/detect_mu2.pt")
    yolo_eval.eval(image_list, img_save=True)

# from ultralytics import YOLO


# class YoloEval:
#     def __init__(self, model_path):
#         self.model = YOLO(model_path)

#     def eval(self, image_path, conf = 0.25, img_save = False):
#         results = self.model.predict(source=image_path, conf=conf)
#         for i, result in enumerate(results):
#             if img_save:
#                 result.save(filename=f'result_{i}.png')  # 결과 이미지 저장


# if __name__ == "__main__":
#     yolo_eval = YoloEval("/home/fred/rokey_ws/detect_mu2.pt")
#     yolo_eval.eval(["/home/fred/rokey_ws/images/아군3.png"], img_save=True)