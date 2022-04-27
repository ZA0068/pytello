import numpy as np
import math
import simpful as sf
import matplotlib.pyplot as plt

class DroneController:
    def __init__(self):
        self.controller = None
    
# setters

    def Setup(self):
        self.SetController()
        self.SetInputs()
        self.SetOutputs()
        self.SetRules()
        
    def SetController(self):
        self.controller = sf.FuzzySystem()


    def SetInputs(self):
        Input_X = []
        Input_X.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = True))
        Input_X.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_right", verbose = True))
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
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far_left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = True))
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far_right", verbose = True))
        lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
        self.GetController().add_linguistic_variable("theta", lv_Theta, verbose = True)

    def SetOutputs(self):       
        Outputs_X = []
        Outputs_X.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "hard_left", verbose = True))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "left", verbose = True))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "soft_left", verbose = True))
        Outputs_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop", verbose = True))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "soft_right", verbose = True))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "right", verbose = True))
        Outputs_X.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "hard_right", verbose = True))
        lv_X = sf.LinguisticVariable(Outputs_X, concept = "Velocity in lateral(x) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_x", lv_X, verbose = True)
        
        Outputs_Y = []
        Outputs_Y.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "ascend hard", verbose = True))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "ascend", verbose = True))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "ascend light", verbose = True))
        Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover", verbose = True))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "descend light", verbose = True))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "descend", verbose = True))
        Outputs_Y.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "descend hard", verbose = True))
        lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_y", lv_Y, verbose = True)
    
    
        Outputs_Z = []
        Outputs_Z.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "fast forward", verbose = True))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "forward", verbose = True))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "slow forward", verbose = True))
        Outputs_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop", verbose = True))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "slow reverse", verbose = True))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "reverse", verbose = True))
        Outputs_Z.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "fast reverse", verbose = True))
        lv_Z = sf.LinguisticVariable(Outputs_Z, concept = "Velocity in longitual(z) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_z", lv_Z, verbose = True)
    
        Outputs_Theta = []
        Outputs_Theta.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "turn hard_right", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "turn right", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "turn slightly_right", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "no turn", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "turn slightly_left", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "turn left", verbose = True))
        Outputs_Theta.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "turn hard_left", verbose = True))
        lv_Theta = sf.LinguisticVariable(Outputs_Theta, concept = "Angular velocity in yaw(θ) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("angular_velocity_θ", lv_Theta, verbose = True)
    
    def SetRules(self):
        Rule1 = "IF (x IS far_left) THEN (velocity_x is hard_right)"
        Rule2 = "IF (x IS left) THEN (velocity_x is right)"
        Rule3 = "IF (x IS slightly_left) THEN (velocity_x is soft_right)"
        Rule4 = "IF (x IS center) THEN (velocity_x is stop)"
        Rule5 = "IF (x IS slightly_right) THEN (velocity_x is soft_left)"
        Rule6 = "IF (x IS right) THEN (velocity_x is left)"
        Rule7 = "IF (x IS far_right) THEN (velocity_x is hard_left)"
        self.GetController().add_rules([Rule1, Rule2, Rule3, Rule4, Rule5, Rule6, Rule7])
        
    def SetVar(self, name, value):
        self.GetController().set_variable(name, value)
        
    def SetX(self, x):
        self.SetVar("x", x)
    
    def SetY(self, y):
        self.SetVar("y", y)
        
    def SetZ(self, z):
        self.SetVar("z", z)
        
# getters
    
    def GetController(self) -> sf.FuzzySystem:
        return self.controller

    def GetX(self):
        return 0.0
    
    def GetY(self):
        return 0.0