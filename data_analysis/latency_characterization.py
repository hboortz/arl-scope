from matplotlib import pyplot as plt
import random

# The relevant pictures live on the SCOPE drive in Photos/LightbridgeLatencyTests
# The pictures at 12:20 and 13:00 are out of line-of-sight
# The pixelation lasted less than 0.3 seconds at each occurrence
# The time between different pixelation incidents ranged from 3s to 20s, with an
# average of approximately 10s. We haven't done a thorough characterization yet.

INITIAL_ERROR = 0.015 # seconds

latencies = [0.45, 0.40, 0.42, 0.38, 0.36, 0.40, 0.36, 0.38, 0.38, 0.38, 0.39, 0.34, 0.39]
corrected_latencies = [lag - INITIAL_ERROR for lag in latencies]
jittered_x_coordinates = [0 + random.uniform(-0.02, 0.02) for each in corrected_latencies]

figure = plt.figure(1)

boxplot_axes = figure.add_subplot(1, 2, 1)
boxplot = boxplot_axes.boxplot(corrected_latencies, showmeans=True)
boxplot_axes.set_ylim(0.32, 0.44)
boxplot_axes.set_title('Boxplot of Data')
boxplot_axes.set_ylabel('Latency (seconds)')
boxplot_axes.set_xlabel('275 meters at 480p')
boxplot_axes.tick_params(
	axis='x', which='both', bottom='off', top='off', labelbottom='off'
)
boxplot_axes.grid(which='major', axis='y')

scatter_axes = figure.add_subplot(1, 2, 2)
scatterplot = scatter_axes.scatter(jittered_x_coordinates, corrected_latencies)
scatter_axes.set_xlim(-0.2, 0.2)
scatter_axes.set_ylim(0.32, 0.44)
scatter_axes.set_title('Raw Data (jittered on x axis)')
scatter_axes.set_xlabel('275 meters at 480p')
scatter_axes.tick_params(
	axis='x', which='both', bottom='off', top='off', labelbottom='off'
)
scatter_axes.grid(which='major', axis='y')
plt.show()