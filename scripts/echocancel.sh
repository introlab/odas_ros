#!/usr/bin/env bash

## Creates an echo cancelled source from source $1

# $1 is the sound source name from "pacmd list-sources | grep 'name:'"
# $2 is the sound sink name from "pacmd list-sinks | grep 'name:'"
#   -> Set to "__default" to use the default sink
# $3 is a name for the resulting names, and it should not contain spaces
#   -> Echo cancelled source name and description: $3_ec
#   -> Echo cancelled sink name and description: $3_ec_sink
# $4 is volume in percent to which the sink should be set

source_name="$3_ec"
sink_name="$3_ec_sink"
volume="$4"

pactl unload-module module-echo-cancel 2> /dev/null

if [ "$2" = "__default" ]
then
    sink_master_opt=""
else
    sink_master_opt="sink_master=$2"
fi

pactl load-module module-echo-cancel source_master="$1" $sink_master_opt aec_method=webrtc source_name=$source_name sink_name=$sink_name use_master_format=yes source_properties=device.description=$source_name sink_properties=device.description=$sink_name 1> /dev/null

rosrun odas_ros movesinks.sh $sink_name
rosrun odas_ros setvolume.sh $source_name $volume

trap "echo \"Unloading module-echo-cancel\"; pactl unload-module module-echo-cancel" INT QUIT KILL TERM 

sleep infinity
