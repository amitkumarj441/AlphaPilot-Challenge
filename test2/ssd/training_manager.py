import numpy as np
import cv2

from .prior_util import PriorUtil

class TrainingManager(object):
    def __init__(self, experimentname, epoch=10, batch_size=32):
        super().__init__()
        # class attributes
        self.experimentname = experimentname
        self.epoch = epoch
        self.batch_size = batch_size
        self.model = None
        self.weightpath = None
        self.loss = None
        self.callbacks = []

    def setModel(self, model, weightpath=None):
        self.model = model
        self.weightpath = weightpath
        self.prior = PriorUtil(self.model)
        if weightpath:
            self.model.load_weights(weightpath, by_name=True)
        a, b, c, d = model.layers[0].input_shape
        self.input_shape = (b, c)
        return

    def predict(self, image, confidence_threshold=0.25, keep_top_k=1, pad=False):
        img_w, img_h = image.shape[:2]
        image = cv2.resize(image, self.input_shape)
        image = image.reshape(1, image.shape[0], image.shape[1], image.shape[2])
        results = self.model.predict(image)
        results = self.prior.decode(results[0], confidence_threshold=confidence_threshold, keep_top_k=keep_top_k)
        if results.shape[0] > 0:
            results = results[:,4:12]
            results = np.concatenate([results, np.ones([results.shape[0],1])], axis=1) # TODO change confidence setting
            results = np.multiply(results, np.append(np.tile(np.array([img_w, img_h]), 4), [1.]))
        return results
