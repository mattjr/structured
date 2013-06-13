set(files  plycullmaxx.cc  plycullmaxx_inv.cc  plymerge.cc)
foreach(file ${files})

GET_FILENAME_COMPONENT(EN ${file} NAME_WE )
  add_executable( ${EN} ${file} )
  TARGET_LINK_LIBRARIES( ${EN} ply linear)

endforeach()
