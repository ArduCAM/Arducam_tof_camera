import sys
import cv2
import numpy as np
import ArduCamDepthCamera as ac

print(dir(ac))

print(dir(ac.ArducamCamera()))
if __name__ == "__main__":
    cam = ac.ArducamCamera()
    cam.initialize(ac.TOFOutput.RAW)
    cam.start()
    while True:
        frame = cam.requestFrame(200)
        buf = frame.getRawData()
        cam.releaseFrame(frame)
        # raw = cv2.Mat(buf)
        # print(buf)
        cv2.imshow("window", buf.astype(np.float32))
        key = cv2.waitKey(1)
        if key == ord("q"):
            exit_ = True
            cam.stop()
            sys.exit(0)
