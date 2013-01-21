#!/usr/bin/python
import cv
import sys
original = cv.LoadImageM(sys.argv[1])
thumbnail = cv.CreateMat(original.rows / int(sys.argv[3]), original.cols / int(sys.argv[4]), cv.CV_8UC3)
cv.Resize(original, thumbnail,cv.CV_INTER_NN)
cv.SaveImage(sys.argv[2],thumbnail)
