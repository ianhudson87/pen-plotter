import matplotlib.pyplot as plt
import numpy as np

traceFile="trace.log"

with open(traceFile) as f:
    lines = [line.rstrip('\n') for line in f]

labels = []

xVals = []
yVals = []

xTargetVals = []
yTargetVals = []

i=0
for line in lines:
    i += 1
    words = line.split()
    if words[0] == "[pos]" and words[4]=="0":
        x = float(words[1])
        y = float(words[2])
        xVals.append(x)
        yVals.append(y)
        labels.append([i, (x, y)])

    if words[0] == "nextCoords:" and words[4]=="0":
        x = float(words[1])
        y = float(words[2])
        xTargetVals.append(x)
        yTargetVals.append(y)
        labels.append([i, (x, y)])

xpoints = np.array(xVals)
ypoints = np.array(yVals)

xtargetpoints = np.array(xTargetVals)
ytargetpoints = np.array(yTargetVals)

# print(xtargetpoints)
# print(ytargetpoints)

plt.scatter(xpoints, ypoints, s=1)

plt.scatter(xtargetpoints, ytargetpoints, s=1)

for label in labels:
    plt.annotate(label[0], (label[1][0], label[1][1]))

# plt.xticks([-15, -10, 5, 0])

plt.show()