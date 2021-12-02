#!/bin/bash

Google_sheet_url="https://docs.google.com/spreadsheets/d/1Td7fD1Ya5e8nmj6GnjdUKj9FcWEayRHbugSZnfPzyxw/edit#gid=0"
mqtt_broker_url="http://www.hivemq.com/demos/websocket-client/"

# Print some message
echo "** Opening $mqtt_broker_url and $mqtt_broker_url in Chrome **"

# Use google-chrome to open the two URLs in separate windows
google-chrome --profile-directory="Profile 1" --new-window $Google_sheet_url $mqtt_broker_url

