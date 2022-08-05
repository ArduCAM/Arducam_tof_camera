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

seletRect_x1 = 0
seletRect_y1 = 0
seletRect_x2 = 0
seletRect_y2 = 0

followRect_x1 = 0
followRect_y1 = 0
followRect_x2 = 0
followRect_y2 = 0

def on_mouse(event, x, y, flags, param):
    global followRect_x1,followRect_y1,followRect_x2,followRect_y2
    global seletRect_x1,seletRect_y1,seletRect_x2,seletRect_y2
    # if (x < 4 or x > 251 or y < 4 or y > 251):
    #     return
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        seletRect_x1 = x - 4 if x - 4 > 0 else 0
        seletRect_y1 = y - 4 if y - 4 > 0 else 0
        seletRect_x2 = x + 4 if x + 4 < 240 else 240
        seletRect_y2=  y + 4 if y + 4 < 180 else 180
    else:
        followRect_x1 = x - 4 if x - 4 > 0 else 0
        followRect_y1 = y - 4 if y - 4 > 0 else 0
        followRect_x2 = x + 4 if x + 4 < 240 else 240
        followRect_y2 = y + 4 if y + 4 < 180 else 180


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.initialize(ac.TOFOutput.DEPTH) != 0 :
        print("initialization failed")
    if cam.start() != 0 :
        print("Failed to start camera")
    cv2.namedWindow("preivew", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preivew",on_mouse)
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            amplitude_buf[amplitude_buf<0] = 0
            amplitude_buf[amplitude_buf>255] = 255
            cam.releaseFrame(frame)

            cv2.imshow("preivew_amplitud", amplitude_buf.astype(np.uint8))

            result_image = process_frame(depth_buf,amplitude_buf)
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            cv2.rectangle(result_image,(seletRect_x1,seletRect_y1),(seletRect_x2,seletRect_y2),(128,128,128), 1)
            cv2.rectangle(result_image,(followRect_x1,followRect_y1),(followRect_x2,followRect_y2),(255,255,255), 1)
            print("select Rect distance:",np.mean(depth_buf[seletRect_x1:seletRect_x2,seletRect_y1:seletRect_y2]))
            cv2.imshow("preivew",result_image)

            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                sys.exit(0)
