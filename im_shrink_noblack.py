#!/usr/bin/python

# structured - Tools for the Generation and Visualization of Large-scale
# Three-dimensional Reconstructions from Image Data. This software includes
# source code from other projects, which is subject to different licensing,
# see COPYING for details. If this project is used for research see COPYING
# for making the appropriate citations.
# Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
#
# This file is part of structured.
#
# structured is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# structured is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with structured.  If not, see <http://www.gnu.org/licenses/>.

import cv
import sys
original = cv.LoadImageM(sys.argv[1])
thumbnail = cv.CreateMat(original.rows / int(sys.argv[3]), original.cols / int(sys.argv[4]), cv.CV_8UC3)
cv.Resize(original, thumbnail,cv.CV_INTER_NN)
cv.SaveImage(sys.argv[2],thumbnail)
