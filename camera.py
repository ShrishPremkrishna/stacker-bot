import cv2
import time
from edge_impulse_linux.image import ImageImpulseRunner

runner = None
model = 'obj-det.eim'
pi_camera = VideoCamera() 
dir_path = os.path.dirname(os.path.realpath(__file__))
modelfile = os.path.join(dir_path, model)
print('MODEL: ' + modelfile)

class VideoCamera(object):
    def __init__(self):
        self.runner = ImageImpulseRunner(modelfile)
        model_info = runner.init()
        print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
        self.labels = model_info['model_parameters']['labels']
        self.camera = cv2.VideoCapture(0)

    def __del__(self):
        self.camera.release()
        cv2.destroyAllWindows()
        if (self.runner):
            self.runner.stop()

    def get_frame(self):
        res, img = runner.classifier(0)

        # if "classification" in res["result"].keys():
        #     print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
        #     for label in self.labels:
        #         score = res['result']['classification'][label]
        #         print('%s: %.2f\t' % (label, score), end='')
        #     print('', flush=True)

        if "bounding_boxes" in res["result"].keys():
            print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
            for bb in res["result"]["bounding_boxes"]:
                print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

        ret, jpeg = cv2.imencode('.jpg', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        return jpeg.tobytes()