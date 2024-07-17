from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=16, imgsz=640, epochs=400, workers=1, device=[5,6,7], verbose=True)
