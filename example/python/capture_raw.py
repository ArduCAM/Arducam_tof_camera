import sys
import cv2
import numpy as np
import ArduCamDepthCamera as ac

if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.init(ac.TOFConnect.USB,ac.TOFOutput.RAW,0) != 0 :
        print("initialization failed")
    if cam.start() != 0 :
        print("Failed to start camera")

    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            buf = frame.getRawData()
            cam.releaseFrame(frame)
            cv2.imshow("window", buf.astype(np.float32))
            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
