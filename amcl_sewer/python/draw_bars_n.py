#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import sys


def load_stats_localization(name, first, last):
    
    for i in range(first,last):
        i_ = i - first
        realname = name + str(i) +'.txt';
        print "Loading: ", realname 
        s = np.loadtxt(realname)
        if i == first:
            data = np.zeros((last - first, len(s)))
        for j in range(len(s)):
            data[i_, j ] = s[j,6]
    std_d = np.std(data,axis=0)
    return np.mean(data,axis=0), std_d

if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.stderr.write("Usage: {0} <stats (xN)> <first> <last>\n".format(sys.argv[0]))
        sys.exit(1)

    n_meas = len(sys.argv) - 3
    print "N_meas = ", n_meas
    first = int(sys.argv[n_meas + 1])
    last = int(sys.argv[n_meas + 2])
    # std_d1 = np.zeros(4)
    # std_d2 = np.zeros(4)
    # std_d3 = np.zeros(4)
    d1, std_d1 = load_stats_localization(sys.argv[1], int(first), int(last))
    d2, std_d2 = load_stats_localization(sys.argv[2], int(first), int(last))
    d3, std_d3 = load_stats_localization(sys.argv[3], int(first), int(last))
    print std_d1
    # fig1, ax1 = plt.subplots()
    # ax1.boxplot(data)
    
    # plt.setp(ax1.get_yticklabels(), fontsize=22)
    # plt.xlabel('Manhole number', fontsize=22)
    # plt.ylabel('Localization error (m)', fontsize=22)
    # plt.show()
    
    ind = np.arange(len(d1))  # the x locations for the groups
    width = 0.2  # the width of the bars

    fig, ax = plt.subplots()
    plt.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        ) # labels along the bottom edge are off
    
    rects1 = ax.bar(ind - width, d1, width, yerr=std_d1)
    rects2 = ax.bar(ind , d2, width,yerr=std_d2)
    rects3 = ax.bar(ind + width, d3, width,yerr=std_d3)
# Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Localization error (m)', fontsize=22)
    ax.set_xticks(ind)
    ax.set_xticklabels(('A', 'B', 'B', 'A'), fontsize=22)
    plt.setp(ax.get_yticklabels(), fontsize=22)
    # ax.legend()

    plt.show()
