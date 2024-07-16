from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=12, imgsz=640, epochs=400, workers=2, device=[0,1], verbose=True)
