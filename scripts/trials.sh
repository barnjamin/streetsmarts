#!/bin/bash




BINARY=/home/ben/streetsmarts/build/bin/client
DECIMATE=(1.0 2.0 3.0)
FPS=(30 60)

$BINARY --session_prefix "vanilla"

for fps in "${FPS[@]}"
do
    echo "$BINARY --fps $fps"
    $BINARY --fps $fps --session_prefix "fps_$fps" --frames_per_fragment=$fps
done

for dec in "${DECIMATE[@]}"
do
    echo "$BINARY --use_filter --dec-mag $dec"
    $BINARY --use_filter --dec-mag $dec --session_prefix "dec_$dec"
done


echo "$BINARY --exposure 166 --wbalance 4600"
$BINARY --exposure 166 --wbalance 4600  --session_prefix "nobalance"

