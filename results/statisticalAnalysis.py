#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Very light (numpy not needed) script that compute the benchmark statistical indicators."""

import sys
import math

print("")
print("STATISTICAL ANALYSIS:")
print("")

with open(sys.argv[1]) as textFile:
    data_ligne = [line.split() for line in textFile] #initial storing of all results in data_ligne

    # create two array data_success and data_failed that store the results as floats,
    # tranpose the initial data and store only the results when the test pass
    # compute the sucess rate simultaneously
    # and compute the average flying speed
    data_success = [[] for _ in range(len(data_ligne[0])+1)]
    data_failed = []
    sucessNumber = 0;
    for i in range(len(data_ligne)):
        sucessNumber += float(data_ligne[i][0])
        if float(data_ligne[i][0]) == 1:
            for j in range(len(data_ligne[i])):
                data_success[j].append(float(data_ligne[i][j]))
            data_success[len(data_ligne[i])].append(data_success[2][len(data_success[2])-1]/data_success[3][len(data_success[3])-1])
        if float(data_ligne[i][0]) == 0:
                data_failed.append(float(data_ligne[i][len(data_ligne[i])-1]))
    
    if sucessNumber == 0:
        print("ONLY UNSUCESSFULL FLIGHTS OCCURED.")
        print("The secondary indicators canno't be computed in this case.")
        sys.exit("")

    # compute and store the mean of all indicators
    means = [0.0] * 8
    means[0] = sucessNumber/float(len(data_ligne))
    for i in range(1, 7):
        means[i] = sum(data_success[i])/float(len(data_success[i]))
    means[7] = sum(data_failed)/float(len(data_failed))
        
    # compute the 99% confidance interval on the collision probability (see chernoff bound) 
    estError = math.sqrt(math.log1p(2/0.01)/(2*len(data_ligne)))
    lowsucessbound = max(0, means[0] - estError)
    highsucessbound = min(1, means[0] + estError)

    # Sort the data and compute the decile Ids for both sucess and failed data vectors
    for indic in data_success:
        indic.sort()
    idFirstDecile = int(len(data_success[0]) / 10 )
    idLastDecile = int(len(data_success[0]) * 9 / 10)

    data_failed.sort()
    idFailedFirstDecile = int(len(data_failed) / 10 )
    idFailedLastDecile = int(len(data_failed) * 9 / 10)


    print("STATISTICAL SUCESS RATE: %.2f" % means[0])
    print("Over the %i tests you performed, it means the probablity of sucess is in [%.2f, %.2f] with a 99%% condidence"  %(len(data_ligne), lowsucessbound, highsucessbound))
    print("Secondary Indicators Format : 'Name : Mean [First Decile, Ninth Decile]'")
    print("Travelled Distance (m) : %.2f [%.2f, %.2f]" %(means[2], data_success[2][idFirstDecile], data_success[2][idLastDecile] ))
    print("Time to complete the Test (s) : %.2f [%.2f, %.2f]" %(means[3], data_success[3][idFirstDecile], data_success[3][idLastDecile]))
    print("Consumed Energy (Wh) : %.2f [%.2f, %.2f]" %(means[4], data_success[4][idFirstDecile], data_success[4][idLastDecile]))
    print("Average Speed (m/s) : %.2f [%.2f, %.2f]" %(means[len(data_ligne[0])], data_success[len(data_ligne[0])][idFirstDecile], data_success[len(data_ligne[0])][idLastDecile]))
    print("Average Linear Distance before collision over failed Tests (m) : %.2f [%.2f, %.2f]" %(means[7], data_failed[idFailedFirstDecile], data_failed[idFailedLastDecile]))
