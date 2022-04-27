
import simpful as sf

fs = sf.FuzzySystem()

Input_Z = []
Input_Z.append(sf.FuzzySet(function=sf.Sigmoid_MF(50, 1), term = "too far", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(40, 5), term = "far", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(35, 5), term = "slightly far", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 5), term = "perfect", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(25, 5), term = "slightly below", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.Gaussian_MF(20, 5), term = "close", verbose = True))
Input_Z.append(sf.FuzzySet(function=sf.InvSigmoid_MF(10, 0.8), term = "too close", verbose = True))
lv_X = sf.LinguisticVariable(Input_Z, concept = "Difference in marker distance in longittual axis", universe_of_discourse = [0, 60])
lv_X.plot(outputfile = "Longtitual Position(Z).png")