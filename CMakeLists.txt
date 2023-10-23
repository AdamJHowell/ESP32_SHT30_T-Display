cmake_minimum_required( VERSION 3.23 )
project( ESP32_SHT30_T_Display )

set( CMAKE_CXX_STANDARD 14 )

include_directories( . )

add_executable( ESP32_SHT30_T_Display
		daughters.h
		ESP32_SHT30_T-Display.ino
		NetworkFunctions.ino
		privateInfo.h )

set_target_properties( ESP32_SHT30_T_Display PROPERTIES LINKER_LANGUAGE CXX )

