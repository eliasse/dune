file(GLOB DUNE_ISBINTERFACE_FILES
  vendor/libraries/ISBInterface/ISBInterface.cpp)

set_source_files_properties(${DUNE_ISBINTERFACE_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS}")

list(APPEND DUNE_VENDOR_FILES ${DUNE_ISBINTERFACE_FILES})
