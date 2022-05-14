from djitellopy import Tello
import cv2 as cv
from Arucodetector import Arucodetector
import simpful as sf
def main():
    # try:
    #     drone = Arucodetector()
    #     drone.Setup()
    #     drone.ConnectDrone()
    #     drone.Run()
    #     # drone.connect()
    #     # drone = Tello()
    #     # drone.streamon()
    #     # while 27 != cv.waitKey(10):
    #         # frame_read = drone.get_frame_read()
    #         # img = frame_read.frame
    #         # cv.imshow("Stream", img)
    #         # print(drone.get_speed_x(), drone.get_speed_y(), drone.get_speed_z())
    # except Exception as e:
    #     print(e)
    # finally:
    #     drone.End()
    #     # cv.destroyAllWindows()
        Outputs_Y = []
        Outputs_Y.append(sf.FuzzySet(points=[[-0.6, 1.0], [-0.4, 0.0]], term = "ascend_hard"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.4, -0.2), term = "ascend"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.4, -0.2, -0.0), term = "ascend_light"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.2, 0.4), term = "descend_light"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.2, 0.4, 0.6), term = "descend"))
        Outputs_Y.append(sf.FuzzySet(points=[[0.4, 0.0], [0.6, 1.0]], term = "descend_hard"))
        lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-0.6, 0.6])
        lv_Y.plot()
if __name__ == '__main__':
    main()
