#!/bin/bash

# Script to set AUX4 to MSP_OVERRIDE mode using CLI commands
# Usage: ./set_aux_cli_robust.sh /dev/serial0

SERIAL_PORT=${1:-/dev/serial0}
BAUD_RATE=115200

echo "Setting AUX4 to MSP_OVERRIDE mode via CLI on $SERIAL_PORT"

# Configure serial port
if ! stty -F "$SERIAL_PORT" $BAUD_RATE raw -echo; then
    echo "Failed to configure serial port $SERIAL_PORT"
    exit 1
fi

echo "Serial port configured at $BAUD_RATE baud"

# Function to send command and get response
send_and_read() {
    local command="$1"
    echo "Sending: $command"
    echo -e "$command\r" > "$SERIAL_PORT"
    sleep 0.5
    timeout 3 cat "$SERIAL_PORT" 2>/dev/null || echo ""
}

# Clear any existing data
timeout 1 cat "$SERIAL_PORT" > /dev/null 2>&1

echo "Attempting to enter CLI mode..."
send_and_read "#"

echo "Waiting for CLI prompt..."
sleep 1

echo "Setting AUX4 to MSP_OVERRIDE mode..."
send_and_read "aux 3 50 0 1000 2000 0 0"

echo "Saving settings..."
send_and_read "save"

echo "Exiting CLI..."
send_and_read "exit"

echo "AUX configuration complete!"
