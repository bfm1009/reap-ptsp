import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import sys

'''
Swarm/violin plot for displaying RRT vs DIRT data.
Author: Bryan McKenney
'''

if (len(sys.argv) > 3):
    # Get filename and what y-value to plot from command line arguments
    filename = sys.argv[1]
    xVal = sys.argv[2]
    yVal = sys.argv[3]

    # Create DataFrame object from csv file
    data = pd.read_csv(filename)

    # Make the background of the plot grey with white lines rather than pure white
    sns.set(style="darkgrid")

    if (len(sys.argv) < 5):
        # Use the data to set up a swarm/violin plot with confidence interval displayed
        g = sns.catplot(x=xVal, y=yVal, kind="violin", inner=None, data=data)
        sns.pointplot(x=xVal, y=yVal, color="c", data=data, ax=g.ax)
        sns.swarmplot(x=xVal, y=yVal, color="k", size=3, data=data, ax=g.ax)
    else:
        # Use the data to set up a scatter plot
        g = sns.relplot(x=xVal, y=yVal, data=data)
        '''g.fig.get_axes()[0].set_xscale('log')
        g.fig.get_axes()[0].set_yscale('log')
        x = np.linspace(0, 100, 2)
        y = x
        plt.plot(x, y, color="r")'''

    # Create a PNG of the resulting plot
    plt.savefig(yVal.replace(" ", "") + ".png")

    # Open the window which displays the resulting plot
    #plt.show()
else:
    print("Missing command line arguments. First should be csv filename,"
       + " second should be what x-value to compare, third should be"
       + " what y-value to compare.")