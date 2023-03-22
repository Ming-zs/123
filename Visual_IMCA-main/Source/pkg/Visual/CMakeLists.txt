add_subdirectory(ArmorFinder)
add_subdirectory(VisualCtrl)
add_subdirectory(ArmorOption)

set(VISUAL_MODULE
        ${VISUAL_CTRL}
        ${ARMOR_OPTION}
        ${ARMOR_FINDER}
        CACHE INTERNAL "THE VISUAL_MODULE")
