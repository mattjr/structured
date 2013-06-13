set(files ply2verts.c  plycomps.c    plycrunch.c  plynormals.c plybbox.c    plyconvert.c  plyflip.c    plyremove.c plybound.c   plyhead.c     plyunshared.c   plyarea.cc      plyxform.cc plyconfthresh.cc
        plyaddinfo.cc    plymirror.cc
        plystrip.cc      plyconfcrop.cc  plysubtract.cc
        plyshared.cc             plydice.cc
        plyplanecrop.cc xyz2ply.cc       plysubdiv.cc
        plyclipbboxes.cc plysubtractlist.cc plyadd.cc)

foreach(file ${files})
GET_FILENAME_COMPONENT(EN ${file} NAME_WE )
  add_executable( ${EN} ${file} )
  TARGET_LINK_LIBRARIES( ${EN} ply linear)

endforeach()
