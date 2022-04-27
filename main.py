
import simpful as sf

fs = sf.FuzzySystem()
Outputs_Theta = []
Outputs_Theta.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "turn hard right", verbose = True))
Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "turn right", verbose = True))
Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "turn slightly right", verbose = True))
Outputs_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "do not turn", verbose = True))
Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "turn slightly left", verbose = True))
Outputs_Theta.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "turn left", verbose = True))
Outputs_Theta.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "turn hard left", verbose = True))
lv_Theta = sf.LinguisticVariable(Outputs_Theta, concept = "Angular velocity in yaw(θ) axis", universe_of_discourse = [-1.0, 1.0])
lv_Theta.plot(outputfile = "Yaw Angular Velocity(θ').png")