#!/bin/bash

./build/bin/capture \
    --frames_per_fragment 60 \
    --fragments 2

./build/bin/refine --session latest 

./build/bin/integrate --session latest 


