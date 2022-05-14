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
<<<<<<< HEAD
        Input_Z = []
        Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(80, 1), term = "too_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(70, 5), term = "far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 5), term = "slightly_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(50, 5), term = "perfect", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(40, 5), term = "slightly_close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 5), term = "close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(20, 1), term = "too_close", verbose = True))
        lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual axis", universe_of_discourse = [0, 100])
=======
        Outputs_Z = []
        Outputs_Z.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "fast_reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "slow_reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "slow_forward"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "forward"))
        Outputs_Z.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "fast_forward"))
        lv_Z = sf.LinguisticVariable(Outputs_Z, concept = "Velocity in longitual(z) axis", universe_of_discourse = [-1.0, 1.0])
>>>>>>> 17be262051eaae23659090db1f08eecd16eecf46
        lv_Z.plot()
if __name__ == '__main__':
    main()
