# ZMK用Smoothstepスクロール軸スナップモジュール

if(CONFIG_ZMK_INPUT_PROCESSOR_SMOOTHSTEP_AXIS_SNAP)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(src/smoothstep_axis_snap_processor.c)
endif()
