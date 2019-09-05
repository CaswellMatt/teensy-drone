import matplotlib.pyplot as plt
import numpy as np
import csv
import collections 


from scipy import signal

with open('data.txt', newline = '') as filter:
    data = csv.reader(filter, delimiter=' ')
    time = []
    outputsFiltered = []
    inputs = []

    filterOutputs = collections.deque([]) 
    filterInputs = collections.deque([]) 
    
    inputCoefficientsB = [ 0.086364,   0.086364]
    outputCoefficientsA = [ 1.00000,  -0.82727]

    print(outputCoefficientsA[0])
    filterOrder = len(inputCoefficientsB)

    for x in data:

        try:
            a = x[7]
        except IndexError:
            pass
            continue

        filterInputs.append(float(x[7]))
        
        inputs.append(float(x[7]))

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
            filterOutputs.append(float(x[7]))


        
        
    print(filterOutputs)



    Fs = 2000  # sampling rate
    Ts = 1.0/Fs # sampling interval
    t = np.arange(0,2,Ts) # time vector


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
    ax[0].axis((0,1,-0.05,0.05))

    ax[1].plot(frq[1:],abs(inputFourier[1:]),'r') # plotting the spectrum
    ax[1].set_xlabel('Freq (Hz)')
    ax[1].set_ylabel('|Y(freq)|')

    
    ax[2].plot(t,filteredY)
    ax[2].set_xlabel('Time')
    ax[2].set_ylabel('Amplitude')
    ax[2].axis((0,1,-0.05,0.05))
    
    ax[3].plot(frq[1:],abs(outputFourier[1:]),'r') # plotting the spectrum
    ax[3].set_xlabel('Freq (Hz)')
    ax[3].set_ylabel('|Y(freq)|')

    plt.show()