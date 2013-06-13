INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/vrip/include)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/vrip/src/linear/)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/vrip/src/libply)
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/vrip/src/softrender/)


ADD_DEFINITIONS("-DLINUX -O6 -funroll-loops -fomit-frame-pointer -ffast-math -DFUNCPROTO -DHAVE_UNISTD_H -DLINUX -D_BOOL -Wno-unused -Wmissing-declarations")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/vrip/bin)
SUBDIRS(src volfill)
