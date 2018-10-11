import matplotlib.pyplot as plt
import sys
import csv


with open(sys.argv[1], "r") as csvfile:
    reader = csv.reader(csvfile, delimiter=",")
    for s in reader:
        print(s[0])
        s = [ float(i) for i in s[1:]]
        #plt.plot(s, '-ko')
        plt.plot(s)
        plt.show()
