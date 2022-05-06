import numpy as np
import math
import simpful as sf
import matplotlib.pyplot as plt
import itertools
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
        output_values = [-1.0, -0.7, -0.3, 0.0, 0.3, 0.7, 1.0]
        left_right_movement = ["hard_left", "left", "slightly_left", "center", "slightly_right", "right", "hard_right"]
        up_down_movement = ["ascending_hard", "ascend", "ascend_light", "hover", "descend_light", "descend", "descend_hard"]
        forward_backward_movement = ["fast_reverse", "reverse", "slow_reverse", "stop", "slow_forward", "forward", "fast_forward"]
        yaw_movement = ["turn_hard_left", "turn_left", "turn_slightly_left", "turn_center", "turn_slightly_right", "turn_right", "turn_hard_right"]
        output_movement_names = left_right_movement + up_down_movement + forward_backward_movement + yaw_movement

        for output_index in range(len(output_movement_names)):
            self.GetController().set_crisp_output_value(output_movement_names[output_index], output_values[output_index%7])
        
        
    def SetRules(self):
        Rules = []
        antecedenct_variables = list(self.GetController()._lvs.keys())
        consequent_variables = ["velocity_x", "velocity_y", "velocity_z", "velocity_theta"]
        consequent_values = list(self.GetController()._crispvalues.keys())
        Rules = self.CreateRules(antecedenct_variables, consequent_variables, consequent_values)
        
        self.GetController().add_rules(Rules, verbose=self.GetLog())

    def CreateRules(self, antecedenct_variables_namelist, consequent_variables_namelist, consequent_values):
        for variable_index in range(len(antecedenct_variables_namelist)):
            for consequent_value in consequent_values:
                Rule = self.CreateRule(antecedenct_variables_namelist[variable_index],
                                       consequent_variables_namelist[variable_index],
                                       consequent_value)
        return Rule

    def CreateRule(self, antecedence_variables, consequence_variables, consequent_value):
        Rule = []
        antecedence = self.CreateAntecendence(antecedence_variables)
        consequence = self.CreateConsequence(consequence_variables)
        self.ApplyRules(antecedence, consequence, Rule)
        return Rule

    def ApplyRules(self, consequence, Rule):
        antecedence = []
        for i in range(len(antecedence)):
            rule = f'IF ({antecedence[i]}) THEN ({consequence[i]})'
            Rule.append(rule)

    def CreateConsequence(self, consequence_variable):
        consequence = []
        for consequence_value in [3,2,1]]:    
            consequence.append(f'{consequence_variable} IS {consequence_value}')
        return consequence

    def CreateAntecendence(self, antecedence, antecedence_variable):
        for antecedence_value in self.GetController().get_fuzzy_sets(antecedence_variable):
            antecedence.append(f'{antecedence_variable} IS {antecedence_value._term}')
        return antecedence            
        
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