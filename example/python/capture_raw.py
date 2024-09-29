import cv2
import numpy as np
import ArducamDepthCamera as ac


def main():
    print("Arducam Depth Camera Demo.")
    print("  SDK version:", ac.__version__)

    cam = ac.ArducamCamera()
    cfg_path = "/home/user/Downloads/Configuration/SIF2618RM_Tof_4_Modes/SIF2618RM_Tof_4_Modes_MIPI_2Lane_RAW12_8b_640x4810.cfg"

    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("initialization failed. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.RAW)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.RawData):
            buf = frame.raw_data
            cam.releaseFrame(frame)

            buf = (buf / (1 << 4)).astype(np.uint8)

            cv2.imshow("window", buf)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    cam.stop()
    cam.close()


if __name__ == "__main__":
    main()
