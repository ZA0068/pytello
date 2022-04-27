
import simpful as sf

fs = sf.FuzzySystem()
Outputs_Y = []
Outputs_Y.append(sf.FuzzySet(points=[[-1.0, 1.0], [-0.8, 0.0]], term = "ascend hard", verbose = True))
Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-1.0, -0.7, -0.4), term = "ascend", verbose = True))
Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(-0.6, -0.3, -0.0), term = "ascend light", verbose = True))
Outputs_Y.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 0.1), term = "hover", verbose = True))
Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.0, 0.3, 0.6), term = "descend light", verbose = True))
Outputs_Y.append(sf.FuzzySet(function=sf.Triangular_MF(0.4, 0.7, 1.0), term = "descend", verbose = True))
Outputs_Y.append(sf.FuzzySet(points=[[0.8, 0.0], [1.0, 1.0]], term = "descend hard", verbose = True))
lv_Y = sf.LinguisticVariable(Outputs_Y, concept = "Velocity in vertical(y) axis", universe_of_discourse = [-1.0, 1.0])
lv_Y.plot(outputfile = "Vertical Velocity(Y').png")