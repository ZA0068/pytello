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
        Input_Z = []
        Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(80, 1), term = "too_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(70, 5), term = "far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 5), term = "slightly_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(50, 5), term = "perfect", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(40, 5), term = "slightly_close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 5), term = "close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(20, 1), term = "too_close", verbose = True))
        lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual axis", universe_of_discourse = [0, 100])
        lv_Z.plot()
if __name__ == '__main__':
    main()
