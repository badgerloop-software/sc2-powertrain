# SC2 CANdapter Diagnostic App

Streamlit utility for monitoring and decoding the SC2 powertrain CAN bus
through an Ewert Energy Systems / Orion BMS CANdapter.

## Features

- Connects to the CANdapter virtual COM port at 250 kbit/s CAN
- Displays every received standard or extended CAN frame
- Decodes powertrain IDs `0x100`, `0x108`, `0x109`, and `0x505`
- Decodes float telemetry on IDs `0x500` through `0x504`
- Shows each known BPS limit and live fault status
- Sends the exact persistent-fault reset command on `0x7EB`
- Provides guarded custom-frame transmission and an offline decoder

## Install and run

From this directory:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py -m streamlit run app.py
```

Close the Orion BMS utility and CANdapter utility before connecting; only one
application can own the COM port at a time.

The default settings are:

- Serial baud: 9600
- CAN bitrate: 250 kbit/s
- Standard 11-bit CAN identifiers

The CANdapter must be wired to the same powered and properly terminated CAN
bus as the powertrain board. The CANdapter DB9 uses pin 3 for CAN High and pin
5 for CAN Low according to the manufacturer's documentation.

## Fault reset behavior

The guarded reset button sends:

```text
ID:      0x7EB
DLC:     8
Payload: 03 7F 20 22 00 00 00 00
```

This arms the current powertrain firmware to clear its persistent BPS latch
on the next powertrain power/reset cycle. An active BPS condition cancels the
pending clear. Estop is transient and is not stored in EEPROM.

## Protocol assumptions

- `0x100`: big-endian Pack SoC in bytes 0–1 at 0.1%/bit
- `0x108`: two big-endian unsigned temperatures in whole degrees Celsius
- `0x109`: two big-endian cell voltages at 0.1 mV/bit, followed by pack
  current magnitude at 0.1 A/bit around a raw zero point of `0x8000`
- `0x505`: byte 0 bit 0 is the combined fault status (`1` means fault)

Keep `protocol.py` synchronized with `include/powertrainConfig.h` whenever
the firmware limits or message definitions change.
