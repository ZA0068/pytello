import cv2 as cv
import numpy as np
import os
import time
import math
import sys
import sys
import traceback
import av
import time
import tellopy

class Arucodetector:
    def __init__(self):
        self.marker_size: int = 0
        self.total_markers_in_dictionary: int = 0
        self.is_drone_connected: bool = False
        self.is_drone_streaming: bool = False
        self.drone: tellopy.Tello = None
        self.container: av.Container = None
        self.RotationMatrix: np.array = None
        self.marker_length: int = 0
        self.font: int = 0



# Setters

    def Setup(self):
        self.SetFlippedMatrix()
        self.SetMarkerLength(10)
        self.SetFont(cv.FONT_HERSHEY_PLAIN)
        self.SetMarkerSize(5)
        self.SetNumbersOfMarkersInDictionary(50)
        self.SetCameraCalibrations()

    def SetNumbersOfMarkersInDictionary(self, totalMarkers) -> None:
        self.total_markers_in_dictionary = totalMarkers;

    def SetFont(self, font):
        self.font = font

    def SetMarkerSize(self, markerSize) -> None:
        self.marker_size = markerSize;

    def SetDrone(self, drone=None) -> None:
        if drone is None:
            self.drone = tellopy.Tello()
        else:
            self.drone = drone

    def SetConnectionStatus(self, status) -> None:
        self.is_drone_connected = status

    def SetStreamingStatus(self, status) -> None:
        self.is_drone_streaming = status

    def ConnectDrone(self, drone=None) -> None:
        try:
            self.SetDrone(drone)
            self.GetDrone().connect()
            self.GetDrone().wait_for_connection(60.0)
            self.SetContainer(3)
            self.SetConnectionStatus(True)
        except Exception as ex:
            self.Exception(ex)

    def Exception(self, ex):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

    def SetContainer(self, retry) -> None:
        while self.container is None and 0 < retry:
            retry -= 1
            try:
                self.container = av.open(self.GetDrone().get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')
            finally:
                self.SetStreamingStatus(True)

    def SetMarkerLength(self, length) -> None:
        self.marker_length: int = length

    def SetFlippedMatrix(self) -> None:
        RotationMatrix = np.zeros((3,3), dtype=np.float32)
        RotationMatrix[0,0] =  1.0
        RotationMatrix[1,1] = -1.0
        RotationMatrix[2,2] = -1.0
        self.rotation_matrix = RotationMatrix

    def SetDistortionCoefficients(self) -> None:
        self.distCoeffs = self.LoadFile('cameraDistortion.txt')

    def SetCameraMatrix(self) -> None:
        self.cameraMatrix = self.LoadFile('cameraMatrix.txt')

    def SetCameraCalibrations(self) -> None:
        self.SetCameraMatrix()
        self.SetDistortionCoefficients()

# Getters

    def GetFlippedMatrix(self) -> np.array:
        return self.rotation_matrix

    def GetCameraMatrix(self) -> np.array:
        return self.cameraMatrix

    def GetDistortionCoefficients(self) -> np.array:
        return self.distCoeffs
    
    def GetCameraMatrix(self) -> np.array:
        return self.cameraMatrix

    def GetDrone(self) -> tellopy.Tello:
        return self.drone

    def GetMarkerLength(self) -> int:
        return self.marker_length
    
    def GetMarkerSize(self) -> int:
        return self.marker_size

    def GetNumbersOfMarkersInDictionary(self) -> int:
        return self.total_markers_in_dictionary

    def GetFont(self) -> int:
        return self.font

    def GetContainer(self) -> av.container:
        return self.container

# Is boolean

    def IsDroneConnected(self) -> bool:
        return self.is_drone_connected
    
    def IsDroneStreaming(self) -> bool:
        return self.is_drone_streaming

# Others

    def LoadFile(self, filename):
        return np.loadtxt(filename, delimiter=',')

    def FindMarkers(self, image) -> tuple:
        gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        key = getattr(cv.aruco, f'DICT_{self.GetMarkerSize()}X{self.GetMarkerSize()}_{self.GetNumbersOfMarkersInDictionary()}')
        dictionary = cv.aruco.Dictionary_get(key)
        parameters = cv.aruco.DetectorParameters_create()
        return cv.aruco.detectMarkers(gray_image, dictionary, parameters=parameters, cameraMatrix = self.GetCameraMatrix(), distCoeff = self.GetDistortionCoefficients())

    def draw(self) -> np.ndarray:
        self.FindMarkers()
        if self.ids is not None:
            cv.aruco.drawDetectedMarkers(self.img, self.corners, self.ids)
            rt_vec = cv.aruco.estimatePoseSingleMarkers(self.corners, 10, self.cameraMatrix, self.distCoeffs)
            for i in range(self.ids.size):
                  cv.aruco.drawAxis(self.img, self.cameraMatrix, self.distCoeffs, rt_vec[0][i,i,:], rt_vec[1][i,i,:], 1)
                  str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(rt_vec[1][0,0,:][0], rt_vec[1][0,0,:][1], rt_vec[1][0,0,:][2])
                  cv.putText(self.img, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv.LINE_AA)


    def Run(self) -> None:
        if self.connected == True:
            # skip first 300 frames
            frame_skip = 300
            while True:
                for frame in self.GetContainer().decode(video=0):
                    if 0 < frame_skip:
                        frame_skip = frame_skip - 1
                        continue
                    start_time = time.time()
                    image = cv.cvtColor(np.array(frame.to_image()), cv.COLOR_RGB2BGR)
                    cv.imshow('Original', image)
                    cv.imshow('Canny', cv.Canny(image, 100, 200))
                    cv.waitKey(1)
                    if frame.time_base < 1.0/60:
                        time_base = 1.0/60
                    else:
                        time_base = frame.time_base
                    frame_skip = int((time.time() - start_time)/time_base)
        else:
            return

    def End(self) -> None:
        self.GetContainer().close()
        self.GetDrone().quit()
        self.SetConnectionStatus(False)
