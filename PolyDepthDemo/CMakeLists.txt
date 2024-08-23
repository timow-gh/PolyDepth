set(HDRS
        model.h
        main.h
)

set(SRCS
        main.cpp
        model.cpp
)

configure_file(../tri_models/bunny_noholes.tri bunny_noholes.tri COPYONLY)
configure_file(../models/bunny40k_1.ani bunny40k_1.ani COPYONLY)
add_executable(polydepth_demo ${SRCS})
target_link_libraries(polydepth_demo PRIVATE PQP freeglut_static C2A::C2A polydepth::polydepth)