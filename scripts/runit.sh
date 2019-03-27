#!/bin/bash

args="--session $1 --max_depth 5 --fragments 200 --frames_per_fragment 30"

$HOME/streetsmarts/build/bin/make_fragments $args
$HOME/streetsmarts/build/bin/refine $args 
$HOME/streetsmarts/build/bin/integrate $args 

