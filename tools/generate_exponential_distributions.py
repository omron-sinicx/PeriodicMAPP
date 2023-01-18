import numpy.random
import sys

number_of_lanes=(int)(sys.argv[1])
scale=1./(float)(sys.argv[2])
max_time=(float)(sys.argv[3])
numpy.random.seed(int(sys.argv[4]))
arrival = []
for lane in range(number_of_lanes):
    current=-1.
    while True:
        nx = numpy.random.exponential(scale)
        current+=1.+nx
        if current >= max_time:
            break
        arrival.append((current, lane))
arrival.sort()
print(len(arrival))
for t in arrival:
    print(t[0], t[1])
