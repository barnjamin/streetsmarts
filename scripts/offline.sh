#!/bin/bash

../build/bin/make_fragments offline_conf.json
../build/bin/refine offline_conf.json
../build/bin/integrate offline_conf.json 
