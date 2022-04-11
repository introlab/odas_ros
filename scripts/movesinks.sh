#!/usr/bin/env bash

## Sets sink $1 as the default, and moves every thing that is already playing to it

# $1 is the sound sink name from "pacmd list-sinks | grep 'name:'"

pactl set-default-sink $1

LC_ALL=C pactl list sink-inputs | grep "Sink Input #" | while read line
do
    pactl move-sink-input `echo $line | cut -f2 -d'#'` $1 1> /dev/null
done
