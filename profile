#### Personal settings

export MY_PORT_NUMBER=7000

#### Settings common for everybody

# Without these, you won't be able to compile the samples
export player_install=/usr/local
export PKG_CONFIG_PATH=${player_install}/lib/pkgconfig:${PKG_CONFIG_PATH}

# Without this, "stage" will not run properly.
export LD_LIBRARY_PATH=${player_install}/lib:${LD_LIBRARY_PATH}


