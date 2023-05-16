import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac

if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.open(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.RAW) != 0 :
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
                cam.close()
                sys.exit(0)
