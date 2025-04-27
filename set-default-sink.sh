#!/bin/bash
sleep 5

SINK="alsa_output.usb-GeneralPlus_USB_Audio_Device-00.analog-stereo"

pactl set-default-sink "$SINK"
pactl set-sink-mute "$SINK" 0
pactl set-sink-volume "$SINK" 100%

