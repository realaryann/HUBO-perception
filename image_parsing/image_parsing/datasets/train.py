from ultralytics import YOLO

model = YOLO('yolov8m.pt')

model.train(data='./data.yml', batch=35, imgsz=640, epochs=400, workers=2, device=[0,1,2,3], verbose=True)
