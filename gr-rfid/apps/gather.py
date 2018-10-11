import matplotlib.pyplot as plt
import sys
import csv

fp = open("preamble.out", "r")
stream = {}
for line in fp.readlines():
    line = line.split()
    stream[int(line[0])] = [ float(i) for i in line[1:] ]
fp.close()

fp = open("bistatic/" + sys.argv[1] + "/" + sys.argv[1], "r")
line = fp.read()
line = line.split()
tsp = [int(i) for i in line]
while tsp.count(0):
    tsp.remove(0)
fp.close()

data_out = open("bistatic/" + sys.argv[1] + "data.csv", "a");
dout = csv.writer(data_out, delimiter=",")
#ndata_out = open("learn/n" + sys.argv[1] + "data.csv", "ab");
#ndout = csv.writer(ndata_out, delimiter=",")
#for i in range(len(stream)):
for key in stream:
    if key in tsp:
        writer = dout
    else:
        continue
        writer = ndout
    writer.writerow([key] + stream[key])
data_out.close()
#ndata_out.close()
