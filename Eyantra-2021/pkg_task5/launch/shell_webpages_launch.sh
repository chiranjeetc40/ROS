#!/bin/bash

Google_sheet_url="https://docs.google.com/spreadsheets/d/1eiBCjUvRonAf48qoqXVig6CW9ri5P6sAdvb2spxLcvM/edit#gid=0"
mqtt_broker_url="http://www.hivemq.com/demos/websocket-client/"

# Print some message
echo "** Opening $mqtt_broker_url and $mqtt_broker_url in Chrome **"

# Use google-chrome to open the two URLs in separate windows
chromium-browser --profile-directory="Profile 1" --new-window $Google_sheet_url $mqtt_broker_url

