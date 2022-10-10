#!/usr/bin/env python
import urllib2  # the lib that handles the url stuff
import math
import rospy
from task2.srv import knn, knnResponse

def readData(link):
    #target_url = http://box.vicos.si/rins/b.txt
    linkData = urllib2.urlopen(link) # it's a file like object and works just like a file
    data = []
    next(linkData)
    for line in linkData: # files are iterable
        split = line.split(", ")
        data.append([float(split[0]), float(split[1]), int(split[2][0])])
    #print(data)
    return data

# returns dot distnace between vectors v1 and v2
def dotDistance(v1, v2):

	distance = 0

	for i in range(len(v1)):
		distance = distance + v1[i]*v2[i]

	return distance

# returns magnitude of vector v
def magnitude(v):

	magnitude = 0

	for i in range(len(v)):
		magnitude = magnitude + v[i]**2

	return math.sqrt(magnitude)

# returns cosine similarity between two vectors
def cosineSimilarity(v1, v2):

	cosDis = dotDistance(v1, v2) / (magnitude(v1)*magnitude(v2))

	return cosDis

def predictKNN(req):
        k = 30
        data = readData(req.url)
        testData = [req.nmb1, req.nmb2]

        distances = []
        for sample in data:
            v1 = [sample[0], sample[1]]
            dist = cosineSimilarity(v1, testData)
            distances.append((dist, sample[2]))
        distances.sort(key=lambda distances:distances[0], reverse=True)

        neighbors = []
        # 0 = r , 1 = g, 2 = b, 3 = y
        colorProbabilities = [0, 0, 0, 0]
        for x in range(k):
            neighbors.append(distances[x])
            colorProbabilities[distances[x][1]] = colorProbabilities[distances[x][1]] + 1

        predictedColor = colorProbabilities.index(max(colorProbabilities))
        print(neighbors)
        print(predictedColor, max(colorProbabilities)/float(k))

        return knnResponse(predictedColor)

if __name__ == "__main__":
    #k = 5
    #testData = [5, 1]
    #link = "http://box.vicos.si/rins/b.txt"

    rospy.init_node('klasifikator', anonymous=True)
    service = rospy.Service('knn_service', knn, predictKNN)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    #predictKNN(testData, link)
