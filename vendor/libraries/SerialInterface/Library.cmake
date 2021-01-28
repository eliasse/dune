file(GLOB DUNE_SERIALINTERFACE_FILES
  vendor/libraries/SerialInterface/SerialInterface.cpp)

set_source_files_properties(${DUNE_SERIALINTERFACE_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS} -lboost_system")

list(APPEND DUNE_VENDOR_FILES ${DUNE_SERIALINTERFACE_FILES})
