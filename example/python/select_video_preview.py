import sys
import cv2
import getopt
import numpy as np
import ArducamDepthCamera as ac

MAX_DISTANCE = 4


def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect()

followRect = UserRect()

def on_mouse(event, x, y, flags, param):
    global selectRect,followRect
    
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - 4 if x - 4 > 0 else 0
        selectRect.start_y = y - 4 if y - 4 > 0 else 0
        selectRect.end_x = x + 4 if x + 4 < 240 else 240
        selectRect.end_y=  y + 4 if y + 4 < 180 else 180
    else:
        followRect.start_x = x - 4 if x - 4 > 0 else 0
        followRect.start_y = y - 4 if y - 4 > 0 else 0
        followRect.end_x = x + 4 if x + 4 < 240 else 240
        followRect.end_y = y + 4 if y + 4 < 180 else 180
        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


if __name__ == "__main__":
    
    if(len(sys.argv) < 2):
        usage(sys.argv[0])
        sys.exit()
    opts, arg = getopt.getopt(sys.argv[1:], 'hd:', ['help'])
    for option, value in opts:
        if option in ['-d']:
            if value.isdigit() == True:
                select_video = int(value)
            else:
                usage(sys.argv[0])
                sys.exit()
        else:
            usage(sys.argv[0])
            sys.exit()

    cam = ac.ArducamCamera()
    if cam.open(ac.TOFConnect.USB,select_video) != 0 :
        print("initialization failed")
        sys.exit()
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
        sys.exit()
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview",on_mouse)
    while True:
        frame = cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            amplitude_buf*=(255/1024)
            amplitude_buf = np.clip(amplitude_buf, 0, 255)

            cv2.imshow("preview_amplitude", amplitude_buf.astype(np.uint8))

            result_image = process_frame(depth_buf,amplitude_buf)
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
            cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
            print("select Rect distance:",np.mean(depth_buf[selectRect.start_x:selectRect.end_x,selectRect.start_y:selectRect.end_y]))
            cv2.imshow("preview",result_image)

            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                cam.close()
                sys.exit(0)
