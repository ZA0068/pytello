import numpy as np
import math
import simpful as sf
import matplotlib.pyplot as plt
class DroneController:
    def __init__(self):
        self.controller = None
        self.verbose = False
    
# setters

    def Setup(self):
        self.SetController()
        self.SetInputs()
        self.SetOutputs()
        self.SetRules()
        
    def SetController(self):
        self.controller = sf.FuzzySystem()

    def SetLog(self, verbose = True):
        self.verbose = verbose

    def SetInputs(self):
        Input_X = []
        Input_X.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_left", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = self.GetLog()))
        Input_X.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_right", verbose = self.GetLog()))
        lv_X = sf.LinguisticVariable(Input_X, concept = "Difference in marker distance in lateral(x) axis", universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("x", lv_X, verbose = self.GetLog())
        
        # Input_Y = []
        # Input_Y.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_above", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "above", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_above", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_below", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "below", verbose = self.GetLog()))
        # Input_Y.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_below", verbose = self.GetLog()))
        # lv_Y = sf.LinguisticVariable(Input_Y, concept = "Difference in marker distance in vertical(y) axis", universe_of_discourse = [-100, 100])
        # self.GetController().add_linguistic_variable("y", lv_Y, verbose = self.GetLog())
# 
        # Input_Z = []
        # Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "too_far", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "far", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_far", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "perfect", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_close", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "close", verbose = self.GetLog()))
        # Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "too_close", verbose = self.GetLog()))
        # lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual(z) axis", universe_of_discourse = [0, 60])
        # self.GetController().add_linguistic_variable("z", lv_Z, verbose = self.GetLog())
# 
        # Input_Theta = []
        # Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far_left", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = self.GetLog()))
        # Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far_right", verbose = self.GetLog()))
        # lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
        # self.GetController().add_linguistic_variable("theta", lv_Theta, verbose = self.GetLog())

    def SetOutputs(self):       
        Outputs_X = []
        Outputs_X.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "hard_left", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "left", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "soft_left", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "soft_right", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "right", verbose = self.GetLog()))
        Outputs_X.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "hard_right", verbose = self.GetLog()))
        lv_X = sf.LinguisticVariable(Outputs_X, concept = "Velocity in lateral(x) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_x", lv_X, verbose = self.GetLog())
        
        # Outputs_Y = []
        # Outputs_Y.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "ascend_hard", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "ascend", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "ascend_light", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "descend_light", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "descend", verbose = self.GetLog()))
        # Outputs_Y.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "descend_hard", verbose = self.GetLog()))
        # lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-1.0, 1.0])
        # self.GetController().add_linguistic_variable("velocity_y", lv_Y, verbose = self.GetLog())
    # 
    # 
        # Outputs_Z = []
        # Outputs_Z.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "fast_forward", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "forward", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "slow_forward", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "slow_reverse", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "reverse", verbose = self.GetLog()))
        # Outputs_Z.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "fast_reverse", verbose = self.GetLog()))
        # lv_Z = sf.LinguisticVariable(Outputs_Z, concept = "Velocity in longitual(z) axis", universe_of_discourse = [-1.0, 1.0])
        # self.GetController().add_linguistic_variable("velocity_z", lv_Z, verbose = self.GetLog())
    # 
        # Outputs_Theta = []
        # Outputs_Theta.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "turn_hard_right", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "turn_right", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "turn_slightly_right", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "no_turn", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "turn_slightly_left", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "turn_left", verbose = self.GetLog()))
        # Outputs_Theta.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "turn_hard_left", verbose = self.GetLog()))
        # lv_Theta = sf.LinguisticVariable(Outputs_Theta, concept = "Angular velocity in yaw(theta) axis", universe_of_discourse = [-1.0, 1.0])
        # self.GetController().add_linguistic_variable("angular_velocity_theta", lv_Theta, verbose = self.GetLog())
    
    def SetRules(self):
        Rule = []
        Rule.append("IF (x IS far_left) THEN (velocity_x IS hard_right)")
        Rule.append("IF (x IS left) THEN (velocity_x IS right)")
        Rule.append("IF (x IS slightly_left) THEN (velocity_x IS soft_right)")
        Rule.append("IF (x IS center) THEN (velocity_x IS stop)")
        Rule.append("IF (x IS slightly_right) THEN (velocity_x IS soft_left)")
        Rule.append("IF (x IS right) THEN (velocity_x IS left)")
        Rule.append("IF (x IS far_right) THEN (velocity_x IS hard_left)")
        
        # Rule.append("IF (y IS far_above) THEN (velocity_y IS descend_hard)")
        # Rule.append("IF (y IS above) THEN (velocity_y is descend)")
        # Rule.append("IF (y IS slightly_above) THEN (velocity_y IS descend_light)")
        # Rule.append("IF (y IS center) THEN (velocity_y IS hover)")
        # Rule.append("IF (y IS slightly_below) THEN (velocity_y IS ascend_light)")
        # Rule.append("IF (y IS below) THEN (velocity_y IS ascend)")
        # Rule.append("IF (y IS far_below) THEN (velocity_y IS ascend_hard)")

        # Rule.append("IF (z IS too_far) THEN (velocity_z IS fast_forward)")
        # Rule.append("IF (z IS far) THEN (velocity_z is forward)")
        # Rule.append("IF (z IS slightly_far) THEN (velocity_z IS slow_forward)")
        # Rule.append("IF (z IS perfect) THEN (velocity_z IS stop)")
        # Rule.append("IF (z IS slightly_close) THEN (velocity_z IS slow_reverse)")
        # Rule.append("IF (z IS close) THEN (velocity_z IS reverse)")
        # Rule.append("IF (z IS too_close) THEN (velocity_z IS fast_reverse)")

        # Rule.append("IF (theta IS far_left) THEN (angular_velocity_theta IS turn_hard_left)")
        # Rule.append("IF (theta IS left) THEN (angular_velocity_theta is turn_left)")
        # Rule.append("IF (theta IS slightly_left) THEN (angular_velocity_theta IS turn_slightly_left)")
        # Rule.append("IF (theta IS center) THEN (angular_velocity_theta IS no_turn)")
        # Rule.append("IF (theta IS slightly_right) THEN (angular_velocity_theta IS turn_slightly_right)")
        # Rule.append("IF (theta IS right) THEN (angular_velocity_theta IS turn_right)")
        # Rule.append("IF (theta IS far_right) THEN (angular_velocity_theta IS turn_hard_right)")
        
        self.GetController().add_rules(Rule, verbose=self.GetLog())
        
    def SetVar(self, name, value):
        self.GetController().set_variable(name, value, verbose=self.GetLog())
        
    def SetX(self, x):
        self.SetVar("x", x)
    
    def SetY(self, y):
        self.SetVar("y", y)
        
    def SetZ(self, z):
        self.SetVar("z", z)
        
    def SetTheta(self, theta):
        self.SetVar("theta", theta)
# getters
    
    def GetController(self) -> sf.FuzzySystem:
        return self.controller

    def GetX(self):
        print(self.GetController().inference(["velocity_x"]))
        return 0.0 
    
    def GetY(self):
        return 0.0
    
    def GetZ(self):
        return 0.0
    
    def GetTheta(self):
        return 0.0
    
    def GetLog(self):
        return self.verbose