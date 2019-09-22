import matplotlib.pyplot as plt
import numpy as np
import csv
import collections 


from scipy import signal

with open('data.txt.4.csv', newline = '') as filter:
    data = csv.reader(filter, delimiter=' ')
    time = []
    outputsFiltered = []
    inputs = []

    filterOutputs = collections.deque([]) 
    filterInputs = collections.deque([]) 
    
    inputCoefficientsB = [ 0.0018421   ,0.0047320   ,0.0047320  , 0.0018421]
    outputCoefficientsA = [    1.00000  ,-2.62253   ,2.36927  ,-0.73358]

    print(outputCoefficientsA[0])
    filterOrder = len(inputCoefficientsB)

    for x in data:
        index = 5
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
    t = np.arange(0,5,Ts) # time vector


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
    ax[0].axis((0,5,-10,15))

    ax[1].plot(frq[2:],abs(inputFourier[2:]),'r') # plotting the spectrum
    ax[1].set_xlabel('Freq (Hz)')
    ax[1].set_ylabel('|Y(freq)|')

    
    ax[2].plot(t,filteredY)
    ax[2].set_xlabel('Time')
    ax[2].set_ylabel('Amplitude')
    ax[2].axis((0,5,-10,15))
    
    ax[3].plot(frq[2:],abs(outputFourier[2:]),'r') # plotting the spectrum
    ax[3].set_xlabel('Freq (Hz)')
    ax[3].set_ylabel('|Y(freq)|')

    plt.show()