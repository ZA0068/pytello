from djitellopy import Tello
import cv2 as cv
from Arucodetector import Arucodetector
import simpful as sf
def main():
    try:
        drone = Arucodetector()
        drone.Setup()
        drone.ConnectDrone()
        drone.Run()
    except Exception as e:
        print(e)
    finally:
        drone.End()
        cv.destroyAllWindows()
if __name__ == '__main__':
    main()
