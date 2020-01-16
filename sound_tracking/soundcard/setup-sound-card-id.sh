#!/usr/bin/env sh

RULE_FILE="/lib/udev/rules.d/85-8-sound-usb.rules"

/bin/cat <<EOM >$RULE_FILE
SUBSYSTEM!="sound", GOTO="8_sounds_usb_end"
ACTION!="add", GOTO="8_sounds_usb_end"

DEVPATH=="$1", ATTR{id}="8_sounds_usb"

LABEL="8_sounds_usb_end"
EOM
