import numpy as np
from scipy.fftpack import fft
import sys
import csv

def main():
  commandLineArg = sys.argv[1]

  with open(commandLineArg, newline = '') as motorData:
    reader = csv.reader(motorData, delimiter=' ')
    next(reader, None)

    freq=[]
    for row in reader:
      freq.append(float(row[0]))

    # Number of sample points
    N = 512
    # sample spacing
    T = 1.0 / 1000.0

    y = freq
    yf = fft(y)
    xf = np.linspace(0.0, 1.0/(2.0*T), N//2)
    import matplotlib.pyplot as plt
    plt.plot(xf[1:N//2], 2.0/N * np.abs(yf[1:N//2]))
    plt.grid()
    plt.show()


main()
