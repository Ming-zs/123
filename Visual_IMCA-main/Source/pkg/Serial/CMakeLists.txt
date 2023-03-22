add_subdirectory(FoundationType)
add_subdirectory(SerialCtrl)

set(SERIAL
        ${FoundationType}
        ${SERIALCTRL}
        CACHE INTERNAL "THE SERIAL")