import sys
import cv2
import numpy as np
import ArduCamDepthCamera as ac

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=30] = 0
    amplitude_buf[amplitude_buf>30] = 255

    depth_buf = (1 - (depth_buf/2)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)

    return result_frame 

if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.initialize(ac.TOFOutput.RAW) != 0 :
        print("initialization failed")
    if cam.start() != 0 :
        print("Failed to start camera")
    while True:
        frame = cam.requestFrame(200)
        depth_buf = frame.getDepthData()
        amplitude_buf = frame.getAmplitudeData()
        amplitude_buf[amplitude_buf<0] = 0
        amplitude_buf[amplitude_buf>255] = 255
        cam.releaseFrame(frame)

        cv2.imshow("preivew_amplitud", amplitude_buf.astype(np.float32))

        result_image = process_frame(depth_buf,amplitude_buf)
        result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
        cv2.imshow("preivew",result_image)

        key = cv2.waitKey(1)
        if key == ord("q"):
            exit_ = True
            cam.stop()
            sys.exit(0)
