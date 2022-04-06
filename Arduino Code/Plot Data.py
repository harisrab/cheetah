import os
import matplotlib.pyplot as plt
import numpy as np

fPath = r"Arduino Code/testPar.txt"

def main():
  
  data = np.genfromtxt(fPath, delimiter=", ")

  fig, axs = plt.subplots()
  axs.plot(data[:,0], data[:,1])

  plt.show()


main()