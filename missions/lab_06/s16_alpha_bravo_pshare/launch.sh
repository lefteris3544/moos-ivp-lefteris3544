#!/bin/bash -e
#----------------------------------------------------------
#  Script: launch.sh
#  Author: Michael Benjamin
#  LastEd: May 20th 2019
#----------------------------------------------------------
#  Part 1: Set Exit actions and declare global var defaults
#----------------------------------------------------------
TIME_WARP=1
GUI="yes"
cd "$(dirname "$0")"

for PORT in 9000 9001 9002; do
    if ss -ltn "sport = :$PORT" | grep -q ":$PORT"; then
        echo "launch.sh: Port $PORT is already in use. Close old MOOS processes first."
        exit 1
    fi
done

#----------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#----------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch.sh [SWITCHES] [time_warp]   "
	echo "  --help, -h           Show this help message            " 
	exit 0;
    elif [ "${ARGI}" = "--nogui" ] ; then
	GUI="no"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#----------------------------------------------------------
#  Part 3: Launch the processes
#----------------------------------------------------------
echo "Launching shoreside, alpha and bravo MOOS Communities with WARP:" $TIME_WARP
pAntler shoreside.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
pAntler alpha.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
pAntler bravo.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

uMAC -t shoreside.moos
kill -- -$$
