#!/usr/bin/env bash

## Sets sink $1 as the default, and moves every thing that is already playing to it

# $1 is the sound sink name from "pacmd list-sinks | grep 'name:'"

pactl set-default-sink $1

LC_ALL=C pactl list sink-inputs | awk '
  /^Sink Input #/ {
    if (input && driver != "module-echo-cancel.c") {
      print input
    }
    input = substr($3, 2)
    driver = ""
  }
  /Driver: / { driver = $2 }
  END {
    if (input && driver != "module-echo-cancel.c") {
      print input
    }
  }
' | while read number
do
    pactl move-sink-input $number $1 1> /dev/null
done
