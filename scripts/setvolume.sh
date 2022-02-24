#!/usr/bin/env bash

## Sets the volume for source $1 to $2 percent

# $1 is the sound source name from "pacmd list-sources | grep 'name:'" without the angle brackets <>

max=$(pacmd list-sources | perl -n0777E "say \$1 if /^.*name: <$1>.*?base volume: ([0-9]+).*/s")
value=$(($2 * $max / 100))

if [ -z "$value" ]; then
    echo "Error: Could not find source $1"
    exit 1
fi

pacmd set-source-volume $1 $value
