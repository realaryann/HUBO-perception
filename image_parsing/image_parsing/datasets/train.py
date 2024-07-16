from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=0.95, imgsz=640, epochs=400, workers=3, device=[0,1,2])
