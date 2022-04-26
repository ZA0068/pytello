
import simpful as sf

fs = sf.FuzzySystem()

Input_Theta = []
Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(-70, -60), term = "far left", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "left", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "slightly left", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "slightly right", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "right", verbose = True))
Input_Theta.append(sf.FuzzySet(function=sf.Crisp_MF(60, 70), term = "far right", verbose = True))
lv_Theta = sf.LinguisticVariable(Input_Theta, concept = "Difference in marker angle in yaw/theta axis", universe_of_discourse = [-70, 70])
lv_Theta.plot(outputfile = "Yaw Angle(Theta).png")