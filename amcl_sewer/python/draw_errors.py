#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys


def load_stats_localization(name, first, last):
    
    for i in range(first,last):
        i_ = i - first
        realname = name + str(i) +'.txt';
        s = np.loadtxt(realname)
        # print s
        if i == first:
            data = np.zeros((last - first, len(s)))
        for j in range(len(s[:,6])):
            data[i_,j] = s[j,6]
            
    print data
    
    fig1, ax1 = plt.subplots()
    ax1.boxplot(data)
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.stderr.write("Usage: {0} <stats> <first> <last> \n".format(sys.argv[0]))
        sys.exit(1)
        
    load_stats_localization(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
  

