set(LIB_DIR ${PROJECT_SOURCE_DIR}/vsfm/lib/)
FIND_PACKAGE(GTK2 REQUIRED)
add_executable(VisualSFM dummy.c)
TARGET_LINK_LIBRARIES(VisualSFM ${LIB_DIR}/VisualSFM.a  ${LIB_DIR}/lapack.a  ${LIB_DIR}/blas.a ${LIB_DIR}/libf2c.a ${LIB_DIR}/libjpeg.a ${GTK2_LIBRARIES} gthread-2.0)