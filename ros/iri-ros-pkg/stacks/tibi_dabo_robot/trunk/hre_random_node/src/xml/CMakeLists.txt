#check the existance of the xsd library
IF(EXISTS "/usr/include/xsd/cxx")
   SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_HAVE_XSD" PARENT_SCOPE)
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_HAVE_XSD" PARENT_SCOPE)
   SET(XSD_FOUND TRUE) 
   MESSAGE(STATUS "Found the XML library ... adding support for XML files")
   FIND_LIBRARY(XSD_LIBRARY
      NAMES xerces-c
      PATHS /usr/lib /usr/local/lib)
ELSE(EXISTS "/usr/include/xsd/cxx")
   MESSAGE(STATUS "XML library not found ... it will be impossible to handle XML files")
ENDIF(EXISTS "/usr/include/xsd/cxx")

IF(XSD_FOUND)
   SET(XSD_LIBRARY ${XSD_LIBRARY} PARENT_SCOPE)

   SET(XSD_PATH ${CMAKE_CURRENT_SOURCE_DIR})
   SET(XSD_FILES hri_cfg_file.xsd)

   IF(XSD_FILES)
      FOREACH(xsd_file ${XSD_FILES})
         STRING(REGEX REPLACE "xsd" "cxx" xsd_source ${xsd_file})
         SET(XSD_SOURCES_INT ${XSD_SOURCES_INT} ${XSD_PATH}/${xsd_source})
         SET(XSD_SOURCES ${XSD_SOURCES} ${XSD_PATH}/${xsd_source})
         STRING(REGEX REPLACE "xsd" "hxx" xsd_header ${xsd_file})
         SET(XSD_HEADERS_INT ${XSD_HEADERS_INT} ${XSD_PATH}/${xsd_header})
         SET(XSD_HEADERS ${XSD_HEADERS} ${XSD_PATH}/${xsd_header})
         SET(XSD_PATH_FILES ${XSD_PATH_FILES} ${XSD_PATH}/${xsd_file})
      ENDFOREACH(xsd_file)

      SET(XSD_SOURCES ${XSD_SOURCES_INT} PARENT_SCOPE)
      SET(XSD_HEADERS ${XSD_HEADERS_INT} PARENT_SCOPE)

      ADD_CUSTOM_TARGET(xsd_files_gen DEPENDS ${XSD_SOURCES_INT})
      ADD_CUSTOM_COMMAND(
         OUTPUT ${XSD_SOURCES_INT}
         COMMAND xsd cxx-tree --generate-serialization ${XSD_FILES}
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
         DEPENDS ${XSD_PATH_FILES}
         COMMENT "Parsing the xml template file ${XSD_FILES}")

      INSTALL(FILES ${XSD_PATH_FILES} DESTINATION include/iridrivers/xml)
      INSTALL(FILES ${XSD_HEADERS_INT} DESTINATION include/iridrivers/xml)
   ENDIF(XSD_FILES)
ENDIF(XSD_FOUND)
