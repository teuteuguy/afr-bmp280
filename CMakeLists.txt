set(module_name "bmp280")
message("Module: ${module_name}")

if (NOT TARGET common-io-helpers)
    add_subdirectory( afr-common-io-helpers )
endif()

add_library(${module_name})

target_sources(
    ${module_name} 
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/src/bmp280.c"
)

target_include_directories(
    ${module_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
)

target_link_libraries(
    ${module_name} 
    PRIVATE
        AFR::common
        AFR::common_io        
        common-io-helpers
)
