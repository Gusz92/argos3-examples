add_library(semantic_loop_functions MODULE 
  semantic_loop_functions.h
  semantic_qtuser_functions.h
  semantic_loop_functions.cpp
  semantic_qtuser_functions.cpp)

target_link_libraries(semantic_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl)
