from ultralytics import YOLO

model = YOLO('yolov8s.pt')

model.train(data='./data.yml', batch=8, imgsz=640, epochs=300, workers=1)
