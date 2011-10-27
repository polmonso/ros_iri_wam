FIND_PATH(wamik_INCLUDE_DIR wamik.h WAMKinematics.h)

FIND_LIBRARY(wamik_LIBRARY
    NAMES wamik
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iriutils) 

IF (wamik_INCLUDE_DIR AND wamik_LIBRARY)
   SET(wamik_FOUND TRUE)
ENDIF (wamik_INCLUDE_DIR AND wamik_LIBRARY)

IF (wamik_FOUND)
   IF (NOT wamik_FIND_QUIETLY)
      MESSAGE(STATUS "Found wamik library: ${wamik_LIBRARY} and include ${wamik_INCLUDE_DIR}")
   ENDIF (NOT wamik_FIND_QUIETLY)
ELSE (wamik_FOUND)
   IF (wamik_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wamik library")
   ENDIF (wamik_FIND_REQUIRED)
ENDIF (wamik_FOUND)

