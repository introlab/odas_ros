#!/usr/bin/env bash

## Sets the volume for source $1 to $2 percent

# $1 is the sound source name from "pacmd list-sources | grep 'name:'" without the angle brackets <>

max=$(LC_ALL=C pactl list sources | perl -n0777E "say \$1 if /^.*Name: $1.*?Base Volume: ([0-9]+).*/s")

if [ -z "$max" ]; then
    echo "Error: Could not find source $1" 1>&2
    exit 1
fi

value=$(($2 * $max / 100))

if [ -z "$value" ]; then
    echo "Error: Could not find source $1" 1>&2
    exit 1
fi

pactl set-source-volume $1 $value
