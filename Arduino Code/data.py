import matplotlib.pyplot as plt
import numpy as np

fname = r"Arduino Code/parabola.txt"

def main():
  # Extract co-ordinates from the file
  data = np.loadtxt(fname, delimiter=' ')

  # Plot the data
  fig, axs = plt.subplots()
  axs.plot(data[:,0], data[:,1])

  plt.show()
main()