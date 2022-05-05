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
        
        Input_Y = []
        Input_Y.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_above", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "above", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_above", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_below", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "below", verbose = self.GetLog()))
        Input_Y.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_below", verbose = self.GetLog()))
        lv_Y = sf.LinguisticVariable(Input_Y, concept = "Difference in marker distance in vertical(y) axis", universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("y", lv_Y, verbose = self.GetLog())

        Input_Z = []
        Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(50, 1), term = "too_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(40, 5), term = "far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(35, 5), term = "slightly_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 5), term = "perfect", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(25, 5), term = "slightly_close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(20, 5), term = "close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(10, 0.8), term = "too_close", verbose = True))
        lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual axis", universe_of_discourse = [0, 60])
        self.GetController().add_linguistic_variable("z", lv_Z, verbose = self.GetLog())

        Input_Theta = []
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far_left", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = self.GetLog()))
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far_right", verbose = self.GetLog()))
        lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
        self.GetController().add_linguistic_variable("theta", lv_Theta, verbose = self.GetLog())

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

        # self.GetController().set_crisp_output_value("hard_left", -1.0)
        # self.GetController().set_crisp_output_value("left", -0.7)
        # self.GetController().set_crisp_output_value("soft_left", -0.3)
        # self.GetController().set_crisp_output_value("stop", 0.0)
        # self.GetController().set_crisp_output_value("soft_right", 0.3)
        # self.GetController().set_crisp_output_value("right", 0.7)
        # self.GetController().set_crisp_output_value("hard_right", 1.0)

        
        Outputs_Y = []
        Outputs_Y.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "ascend_hard", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "ascend", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "ascend_light", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "descend_light", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "descend", verbose = self.GetLog()))
        Outputs_Y.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "descend_hard", verbose = self.GetLog()))
        lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_y", lv_Y, verbose = self.GetLog())
    
        # self.GetController().set_crisp_output_value("", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)

    
        Outputs_Z = []
        Outputs_Z.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "fast_reverse", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "reverse", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "slow_reverse", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "slow_forward", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "forward", verbose = self.GetLog()))
        Outputs_Z.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "fast_forward", verbose = self.GetLog()))
        lv_Z = sf.LinguisticVariable(Outputs_Z, concept = "Velocity in longitual(z) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("velocity_z", lv_Z, verbose = self.GetLog())
    
        Outputs_Theta = []
        Outputs_Theta.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "turn_hard_right", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "turn_right", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "turn_slightly_right", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "no_turn", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "turn_slightly_left", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "turn_left", verbose = self.GetLog()))
        Outputs_Theta.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "turn_hard_left", verbose = self.GetLog()))
        lv_Theta = sf.LinguisticVariable(Outputs_Theta, concept = "Angular velocity in yaw(theta) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("angular_velocity_theta", lv_Theta, verbose = self.GetLog())

        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
        # self.GetController().set_crisp_output_value("velocity_x", 0.0)
            
        
    def SetRules(self):
        Rules = []
        antecedence = []
        consequence = []
        linguistic_variable_namelist = list(self.GetController()._lvs.keys())
        Rules = self.CreateRules(antecedence, consequence, linguistic_variable_namelist)
        
        self.GetController().add_rules(Rules, verbose=self.GetLog())

    def CreateRules(self, antecedence, consequence, linguistic_variable_namelist):
        for index in range(4):
            antecedence_variable = linguistic_variable_namelist[index]
            consequence_variable = linguistic_variable_namelist[index+4]
            Rule = self.CreateRule(antecedence, consequence, antecedence_variable, consequence_variable)
        return Rule

    def CreateRule(self, antecedence, consequence, antecedence_variable, consequence_variable):
        Rule = []
        self.CreateAntecendence(antecedence, antecedence_variable)
        self.CreateConsequence(consequence, consequence_variable)
        self.ApplyRules(antecedence, consequence, Rule)
        return Rule

    def ApplyRules(self, antecedence, consequence, Rule):
        for i in range(len(antecedence)):
            rule = f'IF ({antecedence[i]}) THEN ({consequence[i]})'
            Rule.append(rule)

    def CreateConsequence(self, consequence, consequence_variable):
        for consequence_value in self.GetController().get_fuzzy_sets(consequence_variable):    
            consequence.append(f'{consequence_variable} IS {consequence_value._term}')

    def CreateAntecendence(self, antecedence, antecedence_variable):
        for antecedence_value in self.GetController().get_fuzzy_sets(antecedence_variable):
            antecedence.append(f'{antecedence_variable} IS {antecedence_value._term}')            
        
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

    def UpdateController(self, name):
        return round(self.GetController().inference([name])[name], 2)

    def GetX(self):
        return self.UpdateController("velocity_x")
    
    def GetY(self):
        return self.UpdateController("velocity_y")
    
    def GetZ(self):
        return self.UpdateController("velocity_z")
    
    def GetTheta(self):
        return self.UpdateController("angular_velocity_theta")
    
    def GetLog(self):
        return self.verbose