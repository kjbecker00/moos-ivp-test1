#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch.sh    
#   Author: Michael Benjamin   
#   LastEd: April 2023
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
RANDSTART="true"
VLAUNCH_ARGS="--auto --sim "
SLAUNCH_ARGS="--auto "

XLAUNCHED="no"
SWIM_REGION="-31,-37:-7,-92:95,-46:72,10"
SWIMMERS=15
UNREGERS=0
VROLES=("rescue" "rescue")

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                          "
	echo "                                                    "
	echo "Options:                                            "
        echo "  --help, -h                                        "
        echo "    Display this help message                       "
        echo "  --verbose, -v                                     "
        echo "    Increase verbosity                              "
	echo "  --just_make, -j                                   " 
	echo "    Just make the targ files, but do not launch.    " 
	echo "  --norand                                          " 
	echo "    Do not randomly generate vpositions.txt.        "
	echo "                                                    "
	echo "  --solo                                            " 
	echo "    Only one rescue vehicle (default is two)        "
	echo "  --rs                                              " 
	echo "    Launch one rescue vehicle, one scout vehicle    "
	echo "  --rs2                                             " 
	echo "    Launch two rescue vehicles, two scout vehicles  "
	echo "                                                    "
	echo "  --mit_small                                       " 
	echo "    Use mit small region for random gen of swimmers "
	echo "  --mit_big                                         "
	echo "    Use mit big region for random gen of swimmers   "
	echo "  --swim_file=<mit_00.txt>                          " 
	echo "    Set the swim file.                              "
	echo "  --swimmers=<15>                                   " 
	echo "    Num registered swimmers randomly placed         "
	echo "  --unreg=<0>                                       " 
	echo "    Num UNregistered swimmers randomly placed       "
	echo "                                                    "
	echo "  -1 :  Short for --swim_file=mit_01.txt            "
	echo "  -2 :  Short for --swim_file=mit_02.txt            "
	echo "  -3 :  Short for --swim_file=mit_03.txt            "
	echo "  -4 :  Short for --swim_file=mit_04.txt            "
	echo "  -5 :  Short for --swim_file=mit_05.txt            "
	echo "  -6 :  Short for --swim_file=mit_06.txt            "
	exit 0;
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="-j"
    elif [ "${ARGI}" = "--norand" -o "${ARGI}" = "-r" ]; then
	RANDSTART="false"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        SLAUNCH_ARGS+=" --nogui "
    elif [ "${ARGI}" = "--auto" ]; then
        XLAUNCHED="yes"

    elif [ "${ARGI:0:11}" = "--max_time=" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"

    elif [ "${ARGI}" = "--solo" ]; then
	VROLES=("rescue")
    elif [ "${ARGI}" = "--rs" -o "${ARGI}" = "-rs" ]; then
	VROLES=("rescue" "scout")
	TMATES=("abe" "abe")
    elif [ "${ARGI}" = "--rs2" -o "${ARGI}" = "-rs2" ]; then
	VROLES=("rescue" "rescue" "scout" "scout")
	TMATES=("abe" "ben" "abe" "ben")
    elif [ "${ARGI}" = "--mit_small" ]; then
	SWIM_REGION="-31,-37:-7,-92:95,-46:72,10"
    elif [ "${ARGI}" = "--mit_big" ]; then
	SWIM_REGION="-65,-49:-22.2,-139.4:119.55,-73.4:76.7,16.95"
    elif [ "${ARGI:0:11}" = "--swimmers=" ]; then
        SWIMMERS="${ARGI#--swimmers=*}"
    elif [ "${ARGI:0:8}" = "--unreg=" ]; then
        UNREGERS="${ARGI#--unreg=*}"

    elif [ "${ARGI:0:12}" = "--swim_file=" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-1" -o "${ARGI}" = "-2" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-3" -o "${ARGI}" = "-4" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    elif [ "${ARGI}" = "-5" -o "${ARGI}" = "-6" ]; then
        SLAUNCH_ARGS+=" ${ARGI}"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

AMT=${#VROLES[@]}

#-------------------------------------------------------------
# Part 4: Generate random starting positions
#-------------------------------------------------------------
vecho "Picking starting position and colors"

if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    pickpos --poly="-2,-8 : 4,-13 : 60,13 : 57,18" --buffer=20  \
	    --amt=$AMT  > vpositions.txt  
fi

# generate randomly placed swimmers
if [ "${RANDSTART}" = "true" -o  ! -f "mit_00.txt" ]; then
    gen_swimmers --poly=$SWIM_REGION --swimmers=$SWIMMERS   \
		 --unreg=$UNREGERS --sep=7 > mit_00.txt
fi

VEHPOS=(`cat vpositions.txt`)
VCOLORS=("yellow" "red" "blue" "green")
VNAMES=("abe" "ben" "cal" "deb")

#-------------------------------------------------------------
# Part 5: Launch the vehicles
#-------------------------------------------------------------
ALL_VNAMES=""
for INDEX in `seq 1 $AMT`;
do
    sleep 0.2
    ARRAY_INDEX=`expr $INDEX - 1`
    START=${VEHPOS[$ARRAY_INDEX]}
    VNAME=${VNAMES[$ARRAY_INDEX]}
    COLOR=${VCOLORS[$ARRAY_INDEX]}
    VROLE=${VROLES[$ARRAY_INDEX]}
    TMATE=${TMATES[$ARRAY_INDEX]}
    
    if [ "${ALL_VNAMES}" != "" ]; then
        ALL_VNAMES+=":"
    fi
    ALL_VNAMES+=$VNAME

    MOOS_PORT=`expr $INDEX + 9000`
    PSHARE_PORT=`expr $INDEX + 9200`
    
    IX_VLAUNCH_ARGS="$VLAUNCH_ARGS $TIME_WARP $VERBOSE $JUST_MAKE "
    if [ "${VROLE}" = "scout" -a "${VNAME}" != "${TMATE}" ]; then
        IX_VLAUNCH_ARGS+=" --tmate=${TMATE} "
    fi
    IX_VLAUNCH_ARGS+=" --start=$START --color=$COLOR "
    IX_VLAUNCH_ARGS+=" --mport=$MOOS_PORT --pshare=$PSHARE_PORT "
    IX_VLAUNCH_ARGS+=" --vname=$VNAME --vrole=$VROLE"

    vecho "Launching: $VNAME"
    vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"

    ./launch_vehicle.sh $IX_VLAUNCH_ARGS 
done

#-------------------------------------------------------------
# Part 6: Launch the Shoreside mission file
#-------------------------------------------------------------
SLAUNCH_ARGS+=" $JUST_MAKE $VERBOSE $TIME_WARP "
vecho "Launching the shoreside. Args: $SLAUNCH_ARGS $TIME_WARP"

./launch_shoreside.sh $SLAUNCH_ARGS 

if [ "${JUST_MAKE}" != "" ]; then
    echo "$ME: Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------------- 
#  Part 7: Unless auto-launched, launch uMAC until mission quit          
#-------------------------------------------------------------- 
if [ "${XLAUNCHED}" = "no" ]; then
    uMAC targ_shoreside.moos 
    kill -- -$$
fi

exit 0
