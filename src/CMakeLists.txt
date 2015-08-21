cmake_minimum_required(VERSION 2.6)

set(ufofilter_SRCS
    ufo-direct-gma-task.c)

file(GLOB ufofilter_KERNELS "kernels/*.cl")

include(ConfigurePaths)
include(PkgConfigVars)

configure_paths(UFO_FILTERS)

find_package(OpenCL REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(UFO ufo>=${PKG_UFO_CORE_MIN_REQUIRED} REQUIRED)
pkg_check_modules(PCILIB pcitool>=0.2 REQUIRED)
pkg_check_variable(ufo plugindir)
pkg_check_variable(ufo kerneldir)

set(UFO_LIBRARIES ${PCILIB_LIBRARIES})

add_definitions("-std=c99 -Wall -Wextra -fPIC")
add_definitions(-DG_LOG_DOMAIN="Ufo")
add_definitions("-DHAVE_GMA
                 -I$ENV{AMDAPPSDKROOT}/include
                 -L$ENV{AMDAPPSDKROOT}/lib/x86_64")

if (CMAKE_COMPILER_IS_GNUCC OR ("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang"))
    add_definitions("-Wcast-align -Winline -Wmissing-declarations "
                    "-Wmissing-prototypes -Wnested-externs -Wno-long-long "
                    "-Wno-missing-field-initializers -Wno-unused-parameter"
                    "-Wredundant-decls -Wshadow -Wstrict-prototypes -Wwrite-strings")
endif()


include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${OPENCL_INCLUDE_DIRS}
                    ${UFO_INCLUDE_DIRS}
                    ${PCILIB_INCLUDE_DIRS})

link_directories(${UFO_LIBRARY_DIRS}
                 ${PCILIB_LIBRARY_DIRS})

foreach(_src ${ufofilter_SRCS})
    # find plugin suffix
    string(REGEX REPLACE "ufo-([^ \\.]+)-task.*" "\\1" task "${_src}")

    # build string to get miscalleanous sources
    string(REPLACE "-" "_" _misc ${task})
    string(TOUPPER ${_misc} _misc_upper)

    # create an option name and add this to disable filters
    set(_misc "${_misc}_misc_SRCS")

    string(REPLACE "-" "" _targetname ${task})
    set(target "ufofilter${_targetname}")

    # build single shared library per filter
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        add_library(${target} MODULE ${ufoaux_SRCS} ${_src} ${${_misc}})
    else()
        add_library(${target} SHARED ${ufoaux_SRCS} ${_src} ${${_misc}})
    endif()

    target_link_libraries(${target} ${UFO_LIBRARIES})

    list(APPEND all_targets ${target})

    install(TARGETS ${target}
            ARCHIVE DESTINATION ${UFO_PLUGINDIR}
            LIBRARY DESTINATION ${UFO_PLUGINDIR})
endforeach()


# copy kernels
foreach(_kernel ${ufofilter_KERNELS})
    install(FILES ${_kernel} DESTINATION ${UFO_KERNELDIR})
endforeach()
