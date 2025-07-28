# Drone MSP Configuration Tool

A C++ application that connects to a drone via MultiWii Serial Protocol (MSP) to configure auxiliary channels and verify flight parameters.

## Features

- **MSP Communication**: Connects to drone via `/dev/serial0` using MSP protocol
- **Aux Channel Detection**: Finds aux channels and identifies MSP_OVERRIDE mode (50)
- **Configuration Management**: Saves MSP override channel to `/tmp/config.ini`
- **Parameter Verification**: Checks and sets required flight parameters
- **EEPROM Save**: Saves settings to drone's persistent memory

## Requirements

- Linux system with serial port access
- C++17 compatible compiler
- CMake 3.10 or higher
- Drone running Betaflight/iNav/Cleanflight with MSP support

## Building

```bash
cd drone_msp_app
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
sudo ./drone_msp_app
```

**Note**: Root privileges may be required for serial port access.

## Configuration

The application verifies and sets the following parameters:

- `thrust_linear` = 0
- `roll_srate` = 67
- `pitch_srate` = 67  
- `yaw_srate` = 67
- `rates_type` = ACTUAL
- `roll_rc_rate` = 7
- `pitch_rc_rate` = 7
- `yaw_rc_rate` = 7
- `msp_override_channels_mask` = 15
- `msp_override_failsafe` = ON

## Output

The MSP override channel is saved to `/tmp/config.ini` in the format:
```ini
[drone]
msp_override_channel=7
```

Where the channel number is calculated as `aux_channel_index + 4`.

## Example Output

```
=== Drone MSP Configuration Tool ===
Connecting to drone via MSP on /dev/serial0

--- Step 1: Finding MSP Override Channel ---
Aux 2 mode 50 channel 3 range 1650-2100
Found MSP_OVERRIDE mode on aux channel 3
Saved MSP override channel: aux 3 -> RC channel 7

--- Step 2: Verifying Parameters ---
All parameters are correct

--- Step 3: Saving Settings ---
Settings saved successfully

=== Configuration Complete ===
✓ MSP Override channel found and saved
✓ Parameters verified and updated  
✓ Settings saved to drone EEPROM
✓ Configuration saved to /tmp/config.ini
```

## Troubleshooting

- Ensure drone is connected and powered
- Check serial port permissions: `sudo chmod 666 /dev/serial0`
- Verify baud rate matches drone configuration (default: 115200)
- Check that MSP is enabled on the drone's configuration