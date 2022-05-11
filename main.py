from djitellopy import Tello
import cv2 as cv
from Arucodetector import Arucodetector

def main():
    try:
        drone = Arucodetector()
        drone.Setup()
        drone.ConnectDrone()
        drone.Run()
        # drone.connect()
        # drone = Tello()
        # drone.streamon()
        # while 27 != cv.waitKey(10):
            # frame_read = drone.get_frame_read()
            # img = frame_read.frame
            # cv.imshow("Stream", img)
            # print(drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z())
    except Exception as e:
        print(e)
    finally:
        drone.End()
        # cv.destroyAllWindows()
if __name__ == '__main__':
    main()
