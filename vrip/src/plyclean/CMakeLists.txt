ADD_EXECUTABLE(plyclean plyclean.cc Mesh.cc plyio.cc undo.cc)
TARGET_LINK_LIBRARIES(plyclean ply m)
