from matplotlib import pyplot as plt
import numpy as np
import cv2
import os
from sklearn.neighbors import KNeighborsRegressor
from sklearn.neighbors import NearestNeighbors
import pickle


#create histograms for training data
folder = '/home/pikanogavicka/ROS/src/color_classificator/red'
dataRed = []
dataGreen = []
dataBlue = []
dataBlack = []
data = []
red = 0
test = []
range = [0, 256, 0, 256, 1, 256]
for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            #histr = cv2.calcHist([img],[0],None,[256],[0,256])
            # plt.plot(histr,color = 'b')
            # plt.xlim([0,256])
            # plt.show()
            #data.append(histr)

            # extract a 3D RGB color histogram from the image,
        	# using 8 bins per channel, normalize, and update
        	# the index
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([imgHSV], [0, 1, 2], None, [8, 8, 8], range)
            hist = cv2.normalize(hist, hist).flatten()
            data.append(hist); test = hist; red = red + 1

folder = '/home/pikanogavicka/ROS/src/color_classificator/green'
green =  0
for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            #histr = cv2.calcHist([img],[0],None,[256],[0,256])
            # plt.plot(histr,color = 'b')
            # plt.xlim([0,256])
            # plt.show()
            #data.append(histr)

            # extract a 3D RGB color histogram from the image,
        	# using 8 bins per channel, normalize, and update
        	# the index
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([imgHSV], [0, 1, 2], None, [8, 8, 8], range)
            hist = cv2.normalize(hist, hist).flatten()
            data.append(hist); green = green + 1

folder = '/home/pikanogavicka/ROS/src/color_classificator/blue'
blue =  0
for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            #histr = cv2.calcHist([img],[0],None,[256],[0,256])
            # plt.plot(histr,color = 'b')
            # plt.xlim([0,256])
            # plt.show()
            #data.append(histr)

            # extract a 3D RGB color histogram from the image,
        	# using 8 bins per channel, normalize, and update
        	# the index
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([imgHSV], [0, 1, 2], None, [8, 8, 8], range)
            hist = cv2.normalize(hist, hist).flatten()
            data.append(hist); blue = blue + 1

folder = '/home/pikanogavicka/ROS/src/color_classificator/black'
black =  0
for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            #histr = cv2.calcHist([img],[0],None,[256],[0,256])
            # plt.plot(histr,color = 'b')
            # plt.xlim([0,256])
            # plt.show()
            #data.append(histr)

            # extract a 3D RGB color histogram from the image,
        	# using 8 bins per channel, normalize, and update
        	# the index
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([imgHSV], [0, 1, 2], None, [8, 8, 8], range)
            hist = cv2.normalize(hist, hist).flatten()
            data.append(hist); black = black + 1

#########################
#   for average histogram calculation
#########################
# averageHistRed = []
# for histogram in dataRed:
#     if (len(averageHistRed) == 0):
#         averageHistRed = dataRed[0]
#     else:
#         averageHistRed = averageHistRed + histogram
# averageHistRed = [x/len(dataRed) for x in averageHistRed]
#
# averageHistGreen = []
# for histogram in dataGreen:
#     if (len(averageHistGreen) == 0):
#         averageHistRed = dataGreen[0]
#     else:
#         averageHistGreen = averageHistGreen + histogram
# averageHistGreen = [x/len(dataGreen) for x in averageHistGreen]
#
# averageHistBlue = []
# for histogram in dataBlue:
#     if (len(averageHistBlue) == 0):
#         averageHistBlue = dataBlue[0]
#     else:
#         averageHistBlue = averageHistBlue + histogram
# averageHistBlue = [x/len(dataBlue) for x in averageHistBlue]
#
# averageHistBlack = []
# for histogram in dataBlack:
#     if (len(averageHistBlack) == 0):
#         averageHistBlack = dataBlack[0]
#     else:
#         averageHistBlack = averageHistBlack + histogram
# averageHistBlack = [x/len(dataBlack) for x in averageHistBlack]
#
# with open('avgHistRed.txt', 'w') as f:
#     for item in averageHistRed:
#         f.write("%s\n" % item)
# f.close()
#
# with open('avgHistGreen.txt', 'w') as f:
#     for item in averageHistGreen:
#         f.write("%s\n" % item)
# f.close()
#
# with open('avgHistBlue.txt', 'w') as f:
#     for item in averageHistBlue:
#         f.write("%s\n" % item)
# f.close()
#
# with open('avgHistBlack.txt', 'w') as f:
#     for item in averageHistBlack:
#         f.write("%s\n" % item)
# f.close()
#
# print(dataRed)



##########################
#   for k newarest neighbours clustering
##########################
X = np.array(data)
#X = X.reshape(-1,1)
y = np.empty([red + blue + green + black, 4])
y[0:red] = [1, 0, 0, 0]
y[red:red + blue] = [0, 1, 0, 0]
y[blue + red:green+red + blue] = [0, 0, 1, 0]
y[red + blue + green:] = [0, 0, 0, 1]
#d = cv2.compareHist(index["doge.png"], hist, cv2.HISTCMP_CHISQR)

#build classifier
neigh = KNeighborsRegressor(n_neighbors=10)
neigh.fit(X, y)

print(neigh.predict([test]))
#print(neigh.predict_proba([[0.9]]))

# save the model to disk
filename = 'finalized_colour_model.sav'
pickle.dump(neigh, open(filename, 'wb'))

# load the model from disk
# loaded_model = pickle.load(open(filename, 'rb'))
# result = loaded_model.score(X_test, Y_test)
# print(result)
