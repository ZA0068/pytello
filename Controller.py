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
        Input_X.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_left"))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left"))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left"))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center"))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right"))
        Input_X.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right"))
        Input_X.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_right"))
        lv_X = sf.LinguisticVariable(Input_X, concept = "Difference in marker distance in lateral(x) axis",universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("x", lv_X)
        
        Input_Y = []
        Input_Y.append(sf.FuzzySet(function=sf.InvSigmoid_MF(-90, 1), term = "far_above"))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "above"))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_above"))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center"))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_below"))
        Input_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "below"))
        Input_Y.append(sf.FuzzySet(function=sf.Sigmoid_MF(90, 1), term = "far_below"))
        lv_Y = sf.LinguisticVariable(Input_Y, concept = "Difference in marker distance in vertical(y) axis", universe_of_discourse = [-100, 100])
        self.GetController().add_linguistic_variable("y", lv_Y)

        Input_Z = []
        Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(80, 1), term = "too_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(70, 5), term = "far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 5), term = "slightly_far", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(50, 5), term = "perfect", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(40, 5), term = "slightly_close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 5), term = "close", verbose = True))
        Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(20, 1), term = "too_close", verbose = True))
        lv_Z = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual axis", universe_of_discourse = [0, 100])
        self.GetController().add_linguistic_variable("z", lv_Z)

        Input_Theta = []
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far_left"))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left"))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly_left"))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center"))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly_right"))
        Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right"))
        Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far_right"))
        lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
        self.GetController().add_linguistic_variable("theta", lv_Theta)

    def SetOutputs(self):       
        Outputs_X = []
        Outputs_X.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "hard_left"))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "left"))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "soft_left"))
        Outputs_X.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.05), term = "stop"))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "soft_right"))
        Outputs_X.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "right"))
        Outputs_X.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "hard_right"))
        lv_X = sf.LinguisticVariable(Outputs_X, concept = "Velocity in lateral(x) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("output_x", lv_X)
        
        Outputs_Y = []
        Outputs_Y.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "ascend_hard"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "ascend"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "ascend_light"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "descend_light"))
        Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "descend"))
        Outputs_Y.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "descend_hard"))
        lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("output_y", lv_Y)
    
    
        Outputs_Z = []
        Outputs_Z.append(sf.FuzzySet(points=[[-0.5, 1.0], [-0.3, 0.0]], term = "fast_reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.5, -0.3, -0.1), term = "reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(-0.3, -0.1, -0.0), term = "slow_reverse"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "stop"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.1, 0.3), term = "slow_forward"))
        Outputs_Z.append(sf.FuzzySet(function=sf.Triangular_MF(0.1, 0.3, 0.5), term = "forward"))
        Outputs_Z.append(sf.FuzzySet(points=[[0.3, 0.0], [0.5, 1.0]], term = "fast_forward"))
        lv_Z = sf.LinguisticVariable(Outputs_Z, concept = "Velocity in longitual(z) axis", universe_of_discourse = [-0.5, 0.5])
        self.GetController().add_linguistic_variable("output_z", lv_Z)
    
        Outputs_Theta = []
        Outputs_Theta.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "turn_hard_right"))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "turn_right"))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "turn_slightly_right"))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "no_turn"))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "turn_slightly_left"))
        Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "turn_left"))
        Outputs_Theta.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "turn_hard_left"))
        lv_Theta = sf.LinguisticVariable(Outputs_Theta, concept = "Angular velocity in yaw(theta) axis", universe_of_discourse = [-1.0, 1.0])
        self.GetController().add_linguistic_variable("output_theta", lv_Theta)
    
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
        
    def CreateControlCurves(self):
        try:
            linguistic_variables, input_variables, output_variables = self.GetLinguisticTerms()
            counter = 0
            for input_variable in input_variables:
                vector, output = self.SetupInputAndOutputVectors(linguistic_variables, 
                                                                 output_variables[counter], 
                                                                 input_variable)
                self.CreateOutputFilesAndPlot(vector, 
                                              output, 
                                              self.Capitalize(input_variable))
                counter += 1
        except Exception as e:
            return e
        finally:
            return "Success!"

    def CreateOutputFilesAndPlot(self, vector, output, input_variable_name):
        self.SaveControlCurvePlot(vector, output, input_variable_name)
        self.WriteIntoTextFile("control_curve_input.txt", input_variable_name, vector)
        self.WriteIntoTextFile("control_curve_output.txt", input_variable_name, output)

    def SetupInputAndOutputVectors(self, linguistic_variables, output_variable, input_variable):
        vector = self.GenerateInputVector(linguistic_variables, input_variable)
        output = self.GenerateOutputVector(output_variable, input_variable, vector)
        return vector, output

    def GetLinguisticTerms(self):
        linguistic_variables = self.GetController()._lvs
        input_variables = list(linguistic_variables)[0:4]
        output_variables = list(linguistic_variables)[4:8]
        return linguistic_variables,input_variables,output_variables

    def GenerateInputVector(self, linguistic_variables, input_variable):
        min, max = linguistic_variables[input_variable]._universe_of_discourse
        vector = np.linspace(min, max, 1 + max - min)
        return vector

    def GenerateOutputVector(self, output_variable, input_variable, vector):
        output = []
        for i in vector:
            self.SetVar(input_variable, i)
            output.append(self.UpdateController(output_variable))
        return output

    def Capitalize(self, input_variable):
        input_variable_name = input_variable.capitalize()
        return input_variable_name

    def SaveControlCurvePlot(self, vector, output, input_variable_name):
        plt.plot(vector, output)
        plt.title(f"{input_variable_name} Control Curve")
        plt.xlabel(f"Target's {input_variable_name} position [cm]")
        plt.ylabel("Target's output velocity [cm/s]")
        plt.legend([input_variable_name], loc="lower right", framealpha=1.0)
        plt.savefig(f"Fuzzy controller images/Mamdani_fuzzy_{input_variable_name}.png")
        plt.close()

    def WriteIntoTextFile(self, filename, variable_name, array):
        fileinput = open(f"{variable_name}_{filename}", "w+")
        fileinput.write(','.join(map(str,array)))
        fileinput.close()

# getters
    
    def GetController(self) -> sf.FuzzySystem:
        return self.controller

    def UpdateController(self, name):
        return round(self.GetController().inference([name])[name], 2)

    def GetX(self):
        return self.UpdateController("output_x")
    
    def GetY(self):
        return self.UpdateController("output_y")
    
    def GetZ(self):
        return self.UpdateController("output_z")
    
    def GetTheta(self):
        return self.UpdateController("output_theta")
    
    def GetLog(self):
        return self.verbose