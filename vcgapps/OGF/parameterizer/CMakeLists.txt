
IF(WIN32)
  ADD_DEFINITIONS(-DPARAMETERIZER_EXPORTS)
ENDIF(WIN32)

AUX_SOURCE_DIRECTORY(algos SOURCES)
AUX_SOURCE_DIRECTORY(commands SOURCES)
AUX_SOURCE_DIRECTORY(common SOURCES)
AUX_SOURCE_DIRECTORY(io SOURCES)
AUX_SOURCE_DIRECTORY(shaders SOURCES)
AUX_SOURCE_DIRECTORY(tools SOURCES)

#GOMDIR(parameterizer)

IF(WIN32)
	GatherProjectFiles(parameterizer src/packages/OGF/parameterizer HeaderFiles)
	ADD_LIBRARY(parameterizer ${SOURCES} ${HeaderFiles})
ELSE(WIN32)
	ADD_LIBRARY(parameterizer ${SOURCES})
ENDIF(WIN32)

ADD_DEPENDENCIES(parameterizer basic gom_basic gom gomgen cells gui  math renderer scene_graph skin )


  TARGET_LINK_LIBRARIES(parameterizer  basic  cells   math    )
