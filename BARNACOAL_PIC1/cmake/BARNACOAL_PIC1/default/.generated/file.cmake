# The following variables contains the files used by the different stages of the build process.
set(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assemble)
set_source_files_properties(${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)
set(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assembleWithPreprocess)
set_source_files_properties(${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_assembleWithPreprocess} PROPERTIES LANGUAGE ASM)
set(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_CheckEvents.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_DeferRecall.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_Framework.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_LookupTables.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_Port.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_PostList.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_Queue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/ES_Timers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/circular_buffer_no_modulo_threadsafe.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/dbprintf.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FrameworkSource/terminal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/BoatComm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/DrivetrainService.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/EventCheckers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/PowerService.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/TestHarnessService0.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../ProjectSource/main.c")
set_source_files_properties(${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile_cpp)
set_source_files_properties(${BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_compile_cpp} PROPERTIES LANGUAGE CXX)
set(BARNACOAL_PIC1_default_default_XC32_FILE_TYPE_link)

# The (internal) path to the resulting build image.
set(BARNACOAL_PIC1_default_internal_image_name "${CMAKE_CURRENT_SOURCE_DIR}/../../../_build/BARNACOAL_PIC1/default/default.elf")

# The name of the resulting image, including namespace for configuration.
set(BARNACOAL_PIC1_default_image_name "BARNACOAL_PIC1_default_default.elf")

# The name of the image, excluding the namespace for configuration.
set(BARNACOAL_PIC1_default_original_image_name "default.elf")

# The output directory of the final image.
set(BARNACOAL_PIC1_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/BARNACOAL_PIC1")
