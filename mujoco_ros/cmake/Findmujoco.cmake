find_path(mujoco_INCLUDE_DIR
    NAMES mujoco.h
    PATHS /home/cyborg/pkgs/mujoco-2.2.2/include
    NO_DEFAULT_PATH
    )

find_library(mujoco_LIBRARIES
    NAMES libmujoco.so.2.2.2
    PATHS /home/cyborg/pkgs/mujoco-2.2.2/lib
    NO_DEFAULT_PATH
    )

if (mujoco_LIBRARIES AND mujoco_INCLUDE_DIR)
    message("Mujoco library: ${mujoco_LIBRARIES}")
    message("Mujoco dir: ${mujoco_INCLUDE_DIR}")
    set(FOUND_mujoco TRUE)
endif(mujoco_LIBRARIES AND mujoco_INCLUDE_DIR)
