import json
import numpy as np
import matplotlib.pyplot as plt

# Data in meters. Success was when the quadcopter armed when told to
test_data = ['arming_great_lawn_3-11-15.json']
titles = {test_data[0]: 'Arming quadcopter on Great Lawn with Lightbridge running'}


def plot_data(dataset, color, label):
    for i in range(len(dataset)):
        random_jitter = 0.3 * (np.random.random() - 0.5)
        if i == 0:
            plt.plot(dataset[i], random_jitter, color + 'o', markersize=15,
                     label=label)
        else:
            plt.plot(dataset[i], random_jitter, color + 'o', markersize=15)


def main():
    for test in test_data:
        fin = open('data/' + test, 'ro')
        data = json.load(fin)
        # data saved as dictionary: {"success":[meters], "failure":[meters]}
        plot_data(data["success"], 'g', 'Armed successfully')
        plot_data(data["failure"], 'r', 'Failure to arm')
        plt.title(titles[test])
        plt.xlabel('Distance from quadcopter (m)')
        plt.ylabel('Jitter for viewing purposes')
        plt.axis([0, 25, -0.4, 0.6])
        plt.legend()
        plt.show()


if __name__ == '__main__':
    main()
