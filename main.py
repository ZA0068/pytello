
import simpful as sf


Input = []
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(-90, 15), term = "Far left", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(-60, 15), term = "Left", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(-30, 15), term = "Slightly left", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(0, 15), term = "center", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(30, 15), term = "Slightly right", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(60, 15), term = "Right", verbose = True))
Input.append(sf.FuzzySet(function=sf.Gaussian_MF(90, 15), term = "Far right", verbose = True))
lv = sf.LinguisticVariable(Input, concept = "Lateral movement", universe_of_discourse = [-100, 100])

lv.plot()