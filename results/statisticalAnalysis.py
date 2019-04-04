#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Very light (numpy not needed) script that compute the benchmark statistical indicators."""

import sys

print("")
print("STATISTICAL ANALYSIS:")
print("")

with open(sys.argv[1]) as textFile:
    data_ligne = [line.split() for line in textFile] #initial storing of all results in data_ligne

    # create a data_float that consider results as float,
    # tranpose the initial data and store only the results when the test pass
    # compute the sucess rate simultaneously
    # and compute the average flying speed
    data_float = [[] for _ in range(len(data_ligne[0])+1)]
    sucessNumber = 0;
    for i in range(len(data_ligne)):
        sucessNumber += float(data_ligne[i][0])
        if float(data_ligne[i][0]) == 1:
            for j in range(len(data_ligne[i])):
                data_float[j].append(float(data_ligne[i][j]))
            data_float[len(data_ligne[i])].append(data_float[2][len(data_float[2])-1]/data_float[3][len(data_float[3])-1])

    if sucessNumber == 0:
        print("ONLY UNSUCESSFULL FLIGHTS OCCURED.")
        print("The secondary indicators canno't be computed in this case.")
        sys.exit("")

    # create a vector to store all indicators
    means = [0.0] * (len(data_ligne[0])+1)
    # finish to compute the sucess probablity and store it as the main indicator
    means[0] = sucessNumber/float(len(data_ligne))
    lowsucessbound = max(0, means[0] - 0.05)
    highsucessbound = min(1, means[0] + 0.05)

    # for all other indicator, compute their means
    for i in range(1, len(means)):
        means[i] = sum(data_float[i])/float(len(data_float[i]))

    # Sort he remaining data and compute the decile Ids
    for indic in data_float:
        indic.sort()
    idFirstDecile = int(len(data_float[0]) / 10 )
    idLastDecile = int(len(data_float[0]) *9 / 10)


    print("STATISTICAL SUCESS RATE: %.2f" % means[0])
    print("Hence, if you did the 1060 tests, The probablity of sucess is in [%.2f, %.2f] with a 99%% condidence"  %(lowsucessbound, highsucessbound))
    print("Secondary Indicators Format : 'Name : Mean [First Decile, Ninth Decile]'")
    print("Travelled Distance (m) : %.2f [%.2f, %.2f]" %(means[2], data_float[2][idFirstDecile], data_float[2][idLastDecile] ))
    print("Time to complete the Test (s) : %.2f [%.2f, %.2f]" %(means[3], data_float[3][idFirstDecile], data_float[3][idLastDecile]))
    print("Consumed Energy (Wh) : %.2f [%.2f, %.2f]" %(means[4], data_float[4][idFirstDecile], data_float[4][idLastDecile]))
    print("Average Speed (m/s) : %.2f [%.2f, %.2f]" %(means[len(data_ligne[0])], data_float[len(data_ligne[0])][idFirstDecile], data_float[len(data_ligne[0])][idLastDecile]))
