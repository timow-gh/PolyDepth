set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        freeglut
        GIT_REPOSITORY "https://github.com/timow-gh/freeglut.git"
        GIT_TAG twies/use_custom_cmake_settings
        GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(freeglut)