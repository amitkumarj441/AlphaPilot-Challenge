# Load libraries
import json
import cv2
import numpy as np
import os

from ssd.models.ssd_model import DSOD512
from ssd.training_manager import TrainingManager

# Implement a function that takes an image as an input, performs any preprocessing steps and outputs a list of bounding box detections and assosciated confidence score.

class GenerateFinalDetections():
    def __init__(self):
        self.seed = 2018
        WEIGHTPATH= os.path.dirname(os.path.realpath(__file__)) + '/weights/weights.h5'
        manager = TrainingManager("predict")
        model = DSOD512(num_classes=2)
        manager.setModel(model, weightpath=WEIGHTPATH)
        self.manager = manager

    def predict(self,img):
        img_w, img_h = np.array(img).shape[:2]
        bb_all = self.manager.predict(img, confidence_threshold=0.01, keep_top_k=1, pad=False)
        return bb_all.tolist()
