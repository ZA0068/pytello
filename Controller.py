import numpy as np
import math
import simpful as sf
import matplotlib.pyplot as plt

class DroneController:
    def __init__(self):
        self.controller = None
    

    def Setup(self):
        self.SetController()
        self.SetInputs()
        
        
    def SetController(self):
        self.controller = sf.FuzzySystem()


    def SetInputs(self):
        Input_X = []
        Input_X.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly right", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far right", verbose = True))
        lv_X = sf.LinguisticVariable(Input_X, concept = "Difference in marker distance in lateral(x) axis", universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("x", lv_X, verbose = True)
        
        Input_Y = []
        Input_Y.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far above", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "above", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly above", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly below", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "below", verbose = True))
        Input_Y.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far below", verbose = True))
        lv_Y = sf.LinguisticVariable(Input_Y, concept = "Difference in marker distance in vertical(y) axis", universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("y", lv_Y, verbose = True)

        Input_Z = []
        Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "too far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "perfect", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly below", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "too close", verbose = True))
        lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual(z) axis", universe_of_discourse = [0, 60])
        self.GetController().add_linguistic_variable("z", lv_Z, verbose = True)

        Input_Theta = []
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly right", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far right", verbose = True))
        lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
        self.GetController().add_linguistic_variable("theta", lv_Theta, verbose = True)

    def GetController(self) -> sf.FuzzySystem:
        return self.controller
