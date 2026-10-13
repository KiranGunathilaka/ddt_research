import os
import torch
import ultralytics

MODEL_TYPE = ".engine" if torch.cuda.is_available() else ""
MODEL_PATH = os.getcwd() + "/models/yolov5n_640"

if MODEL_TYPE and not os.path.exists(MODEL_PATH + MODEL_TYPE):
    model = ultralytics.YOLO(MODEL_PATH + ".pt")
    model.export(format="engine", half=True, batch=2)
