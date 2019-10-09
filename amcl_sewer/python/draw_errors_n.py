#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys


def load_stats_localization(name, first, last, n_exp, n_meas, exp_min, exp_max,data):
    
    for i in range(first,last):
        i_ = i - first
        realname = name + str(i) +'.txt';
        print "Loading: ", realname 
        print "N_exp = ", n_exp
        s = np.loadtxt(realname)
        for j in range(exp_min - 1, exp_max):
            data[i_, (j - exp_min + 1) * n_meas + n_exp ] = s[j,6]

if __name__ == '__main__':
    if len(sys.argv) < 6:
        sys.stderr.write("Usage: {0} <stats (xN)> <first> <last> <manhole_min> <manhole_max>\n".format(sys.argv[0]))
        sys.exit(1)

    n_meas = len(sys.argv) - 5
    print "N_meas = ", n_meas
    first = int(sys.argv[n_meas + 1])
    last = int(sys.argv[n_meas + 2])
    manhole_min = int(sys.argv[n_meas + 3])
    manhole_max = int(sys.argv[n_meas + 4])
    data = np.zeros(( last - first, n_meas*(manhole_max - manhole_min + 1)))
    print "Data length= ", len(data)
    for i in range(0, n_meas):
        load_stats_localization(sys.argv[i + 1], int(first), int(last), i, n_meas, manhole_min, manhole_max, data)
    fig1, ax1 = plt.subplots()
    ax1.boxplot(data)
    plt.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        labelbottom=False) # labels along the bottom edge are off
    
    plt.setp(ax1.get_yticklabels(), fontsize=22)
    plt.xlabel('Manhole number', fontsize=22)
    plt.ylabel('Localization error (m)', fontsize=22)
    plt.show()
