include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(BARNACOAL_PIC1_default_library_list )

# Handle files with suffix s, for group default-XC32
if(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assemble)
add_library(BARNACOAL_PIC1_default_default_XC32_assemble OBJECT ${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assemble})
    BARNACOAL_PIC1_default_default_XC32_assemble_rule(BARNACOAL_PIC1_default_default_XC32_assemble)
    list(APPEND BARNACOAL_PIC1_default_library_list "$<TARGET_OBJECTS:BARNACOAL_PIC1_default_default_XC32_assemble>")
endif()

# Handle files with suffix S, for group default-XC32
if(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assembleWithPreprocess)
add_library(BARNACOAL_PIC1_default_default_XC32_assembleWithPreprocess OBJECT ${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assembleWithPreprocess})
    BARNACOAL_PIC1_default_default_XC32_assembleWithPreprocess_rule(BARNACOAL_PIC1_default_default_XC32_assembleWithPreprocess)
    list(APPEND BARNACOAL_PIC1_default_library_list "$<TARGET_OBJECTS:BARNACOAL_PIC1_default_default_XC32_assembleWithPreprocess>")
endif()

# Handle files with suffix [cC], for group default-XC32
if(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile)
add_library(BARNACOAL_PIC1_default_default_XC32_compile OBJECT ${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile})
    BARNACOAL_PIC1_default_default_XC32_compile_rule(BARNACOAL_PIC1_default_default_XC32_compile)
    list(APPEND BARNACOAL_PIC1_default_library_list "$<TARGET_OBJECTS:BARNACOAL_PIC1_default_default_XC32_compile>")
endif()

# Handle files with suffix cpp, for group default-XC32
if(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile_cpp)
add_library(BARNACOAL_PIC1_default_default_XC32_compile_cpp OBJECT ${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile_cpp})
    BARNACOAL_PIC1_default_default_XC32_compile_cpp_rule(BARNACOAL_PIC1_default_default_XC32_compile_cpp)
    list(APPEND BARNACOAL_PIC1_default_library_list "$<TARGET_OBJECTS:BARNACOAL_PIC1_default_default_XC32_compile_cpp>")
endif()

add_executable(${BARNACOAL_PIC1_default_image_name} ${BARNACOAL_PIC1_default_library_list})

target_link_libraries(${BARNACOAL_PIC1_default_image_name} PRIVATE ${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_link})

# Add the link options from the rule file.
BARNACOAL_PIC1_default_link_rule(${BARNACOAL_PIC1_default_image_name})

# Add bin2hex target for converting built file to a .hex file.
add_custom_target(BARNACOAL_PIC1_default_Bin2Hex ALL
    ${MP_BIN2HEX} ${BARNACOAL_PIC1_default_image_name})
add_dependencies(BARNACOAL_PIC1_default_Bin2Hex ${BARNACOAL_PIC1_default_image_name})

# Post build target to copy built file to the output directory.
add_custom_command(TARGET ${BARNACOAL_PIC1_default_image_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E make_directory ${BARNACOAL_PIC1_default_output_dir}
                    COMMAND ${CMAKE_COMMAND} -E copy ${BARNACOAL_PIC1_default_image_name} ${BARNACOAL_PIC1_default_output_dir}/${BARNACOAL_PIC1_default_original_image_name}
                    BYPRODUCTS ${BARNACOAL_PIC1_default_output_dir}/${BARNACOAL_PIC1_default_original_image_name})
