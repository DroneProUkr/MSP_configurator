#!/bin/bash

# Script to set AUX4 to MSP_OVERRIDE mode using CLI commands
# Usage: ./set_aux_cli.sh /dev/serial0

SERIAL_PORT=${1:-/dev/serial0}
BAUD_RATE=115200

echo "Setting AUX4 to MSP_OVERRIDE mode via CLI on $SERIAL_PORT"

# Function to send CLI command and wait for response
send_cli_command() {
    local command="$1"
    local expected_response="$2"
    
    echo "Sending: $command"
    echo -e "$command\r" > "$SERIAL_PORT"
    
    # Wait a bit for response
    sleep 0.5
    
    # Read response
    response=$(timeout 2 cat "$SERIAL_PORT" 2>/dev/null || echo "")
    echo "Response: $response"
    
    # Check if we got the expected response
    if [[ "$response" == *"$expected_response"* ]]; then
        echo "✓ Command successful"
        return 0
    else
        echo "✗ Command failed or unexpected response"
        return 1
    fi
}

# Open serial port
if ! stty -F "$SERIAL_PORT" $BAUD_RATE raw -echo; then
    echo "Failed to configure serial port $SERIAL_PORT"
    exit 1
fi

echo "Serial port configured at $BAUD_RATE baud"

# Send CLI commands
echo "Entering CLI mode..."
send_cli_command "#" "CLI"

echo "Setting AUX4 to MSP_OVERRIDE mode..."
send_cli_command "aux 3 50 0 1000 2000 0 0" "aux 3 50 0 1000 2000 0 0"

echo "Saving settings..."
send_cli_command "save" "saved"

echo "Exiting CLI..."
send_cli_command "exit" "exit"

echo "AUX configuration complete!"
