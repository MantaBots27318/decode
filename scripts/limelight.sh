#!/bin/sh

# Retrieve absolute path to this script
script=$(readlink -f $0)
scriptpath=`dirname $script`
echo $scriptpath

# Create virtual environment
adb shell "su; sh -s" < ./scripts/routing.sh