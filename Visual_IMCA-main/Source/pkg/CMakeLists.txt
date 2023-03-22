add_subdirectory(Visual)
add_subdirectory(GlobalVar)
add_subdirectory(Serial)
add_subdirectory(Camera)
add_subdirectory(Experi)

set(PKG
        ${EXPERI}
        ${VISUAL_MODULE}
        ${GLOBAL_VAR}
        ${SERIAL}
        ${CAMERA}
        CACHE INTERNAL "THE PKGS")