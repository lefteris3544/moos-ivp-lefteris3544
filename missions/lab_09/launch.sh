#!/bin/bash
#----------------------------------------------------------
#  Lab 09: Autonomous Rescue Challenge, Part 1
#----------------------------------------------------------

TIME_WARP=1
JUST_MAKE="no"
SWIM_FILE="athens_rand.txt"

VNAME="abe"
START_POS="0,-25"
RETURN_POS="0,-25"
VPORT="9001"
PSHARE_PORT="9201"
SHORE_PSHARE="9200"
IP_ADDR="localhost"
SHORE_IP="localhost"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "launch.sh [OPTIONS] [time_warp]"
        echo "  --swim_file=FILE    swimmer file for uFldRescueMgr"
        echo "  --athens            use athens_rand.txt"
        echo "  -r1                 use athens_01.txt"
        echo "  -r2                 use athens_02.txt"
        echo "  --just_make, -j     build target files only"
        echo "  --help, -h          show this help"
        exit 0
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--athens" ]; then
        SWIM_FILE="athens_rand.txt"
    elif [ "${ARGI}" = "-r1" ]; then
        SWIM_FILE="athens_01.txt"
    elif [ "${ARGI}" = "-r2" ]; then
        SWIM_FILE="athens_02.txt"
    elif [[ "${ARGI}" == --swim_file=* ]]; then
        SWIM_FILE="${ARGI#--swim_file=}"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    else
        echo "launch.sh Bad Arg: $ARGI. Exit code 1."
        exit 1
    fi
done

if [ ! -f "$SWIM_FILE" ]; then
    echo "Cannot find swim file: $SWIM_FILE"
    exit 1
fi

nsplug meta_vehicle.moos targ_$VNAME.moos -f WARP=$TIME_WARP \
       VNAME=$VNAME START_POS=$START_POS RETURN_POS=$RETURN_POS \
       VPORT=$VPORT PSHARE_PORT=$PSHARE_PORT VTYPE="kayak"      \
       SHORE_IP=$SHORE_IP SHORE_PSHARE=$SHORE_PSHARE IP_ADDR=$IP_ADDR

nsplug meta_vehicle.bhv targ_$VNAME.bhv -f VNAME=$VNAME \
       START_POS=$START_POS RETURN_POS=$RETURN_POS

nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
       VNAME="shoreside" VPORT="9000" PSHARE_PORT=$SHORE_PSHARE  \
       IP_ADDR=$IP_ADDR SWIM_FILE=$SWIM_FILE

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi

echo "Launching shoreside. WARP is $TIME_WARP. Swim file is $SWIM_FILE"
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.25

echo "Launching vehicle $VNAME. WARP is $TIME_WARP"
pAntler targ_$VNAME.moos >& /dev/null &
echo "Done"

uMAC targ_shoreside.moos
kill -- -$$
