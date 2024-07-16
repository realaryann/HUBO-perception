from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=8, imgsz=640, epochs=400, workers=4, device=[3,4], verbose=True)
