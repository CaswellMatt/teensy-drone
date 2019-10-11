import matplotlib.pyplot as plt
import numpy as np
import csv
import collections 
import sys

from scipy import signal

commandLineArg = sys.argv[1]

with open(commandLineArg, newline = '') as motorData:
    reader = csv.reader(motorData, delimiter=' ')
    next(reader, None)
    time = []
    outputsFiltered = []
    inputs = []

    filterOutputs = collections.deque([]) 
    filterInputs = collections.deque([]) 
    




    inputCoefficientsB = [1,	-4.75371,	10.2799,	-13.76442,	12.88273,	-8.91226,	4.74686,	-1.86773,	0.38872,]
    outputCoefficientsA = [0.17481,	-1.26983,	4.15171,	-7.9731,	9.83288,	-7.9731,	4.15171,	-1.26983,	0.17481,]

    print(outputCoefficientsA[0])
    filterOrder = len(inputCoefficientsB)

    for x in reader:
        index = 1
        try:
            a = x[index]
        except IndexError:
            pass
            continue

        filterInputs.append(float(x[index]))
        
        inputs.append(float(x[index]))

        if (len(filterInputs) == filterOrder):
            filterInputs.popleft()
        
        if len(filterOutputs) == filterOrder:
            
            currentOutput = 0
            
            for i in range(1,filterOrder):

                currentOutput += outputCoefficientsA[i] * filterOutputs[-i]
                
            currentOutput = -currentOutput

            for i in range(filterOrder):
                currentOutput += inputCoefficientsB[i] * filterInputs[-i]

            filterOutputs.append(currentOutput)
            filterOutputs.popleft()
            outputsFiltered.append(currentOutput)
            
        else:
            filterOutputs.append(float(x[index]))
        
        
    print(filterOutputs)



    Fs = 1000  # sampling rate
    Ts = 1.0/Fs # sampling interval
    t = np.arange(0,0.500,Ts) # time vector


    inputsY = inputs[:len(t)]

    n = len(inputsY) # length of the signal
    k = np.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(n//2)] # one side frequency range
    # frq = frq[1:]
    inputFourier = np.fft.fft(inputsY) # fft computing and normalization
    inputFourier = inputFourier[range(n//2)]
   

    filteredY = outputsFiltered[:len(t)]

    # frq = frq[1:]
    outputFourier = np.fft.fft(filteredY) # fft computing and normalization
    outputFourier = outputFourier[range(n//2)]

    # yf = scipy.fftpack.fft(values)

    fig, ax = plt.subplots(4, 1)
    
    ax[0].plot(t,inputsY)
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('Amplitude')
    ax[0].axis((0,0.5,-4,4))

    ax[1].plot(frq[2:],abs(inputFourier[2:]),'r') # plotting the spectrum
    ax[1].set_xlabel('Freq (Hz)')
    ax[1].set_ylabel('|Y(freq)|')

    
    ax[2].plot(t,filteredY)
    ax[2].set_xlabel('Time')
    ax[2].set_ylabel('Amplitude')
    ax[2].axis((0,0.5,-20,20))
    
    ax[3].plot(frq[2:],abs(outputFourier[2:]),'r') # plotting the spectrum
    ax[3].set_xlabel('Freq (Hz)')
    ax[3].set_ylabel('|Y(freq)|')

    plt.show()