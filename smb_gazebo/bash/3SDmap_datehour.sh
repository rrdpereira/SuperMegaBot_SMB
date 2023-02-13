#!/bin/bash
# Launch script in background
./3Dmap_datehour.sh &
# Get its PID
PID=$!
# Wait for 2 seconds
sleep 2
# Kill it
kill $PID

