#!/usr/bin/env bash

## Sets sink $1 as the default, and moves every thing that is already playing to it

# $1 is the sound sink name from "pacmd list-sinks | grep 'name:'"

pacmd set-default-sink $1

pacmd list-sink-inputs | grep index | while read line
do
    pacmd move-sink-input `echo $line | cut -f2 -d' '` $1 1> /dev/null
done
