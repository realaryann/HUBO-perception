from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=8, imgsz=640, epochs=1000, workers=1, device=0)
