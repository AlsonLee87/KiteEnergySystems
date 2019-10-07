set(KITE_DYNAMICS_DIR .)
include_directories(${KITE_DYNAMICS_DIR}/inc/utils)

add_library(kite_utils SHARED src/utils/KiteState.cc)