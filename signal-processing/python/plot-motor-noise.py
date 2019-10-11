import matplotlib.pyplot as plt
import numpy as np
import csv
import sys

commandLineArg = sys.argv[1]

with open(commandLineArg, newline = '') as motorData:
  reader = csv.reader(motorData, delimiter=' ')
  
  columnCount = len(next(reader))


  pairCount = int(columnCount/2)
  print(pairCount)
  fig, ax = plt.subplots(columnCount, 1)
  for pairIndex in range(pairCount):
    freq=[]
    amplitude=[]

    motorData.seek(0)
    next(reader, None)

    for row in reader:
      freq.append(float(row[2*pairIndex]))
      amplitude.append(float(row[2*pairIndex+1]))
  
    ax[2*pairIndex].plot(amplitude)
    ax[2*pairIndex].set_xlabel('Time ' + str(pairIndex))
    ax[2*pairIndex].set_ylabel('Amplitude ' + str(pairIndex))

    ax[2*pairIndex + 1].plot(freq[1:], 'r') # plotting the spectrum
    ax[2*pairIndex + 1].set_xlabel('Freq (Hz) ' + str(pairIndex + 1))
    ax[2*pairIndex + 1].set_ylabel('|Y(freq)| ' + str(pairIndex + 1))

  plt.show()