#! /bin/bash
# usage:
# first make your own copy of template
# cp build_flags_template.sh my_build_flags.sh
# then edit, change or comment something
# nano  my_build_flags.sh
# and source it
# source my_build_flags.sh
 export FLAGS="-DNARODMON_HOST=192.168.0.114"
 export FLAGS="$FLAGS -DNEXT_CONN_TIME=3000"
 export FLAGS="$FLAGS -DPOSTING_INTERVAL=2000"
 export PLATFORMIO_BUILD_FLAGS="$FLAGS"
 unset FLAGS