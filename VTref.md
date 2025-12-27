# VTref Sensing for Blackpill-F4 Black Magic Probe

## Overview

This implementation adds target voltage reference (VTref) sensing capability to the Blackpill-F4 platform when built with `SHIELD=1`. The feature allows the probe to detect if the target is already powered before attempting to supply power, preventing conflicts and potential damage.

## Software Changes

### Modified Files

- **`src/platforms/common/blackpill-f4/blackpill-f4.h`**
  - Added VTREF_PORT (GPIOB) and VTREF_PIN (GPIO0) definitions

- **`src/platforms/common/blackpill-f4/blackpill-f4.c`**
  - Implemented `adc_init()` - configures ADC1 and PB0 as analog input
  - Implemented `platform_target_voltage_sense()` - returns voltage in 0.1V units
  - Implemented `platform_target_voltage()` - returns formatted voltage string
  - Added ADC initialization call in `platform_init()` when `PLATFORM_HAS_POWER_SWITCH` is enabled

### Functionality

- **ADC Channel**: ADC12_IN8 (PB0)
- **Resolution**: 12-bit (0-4095)
- **Voltage Range**: 0-3.3V
- **Detection Threshold**: 0.5V (POWER_CONFLICT_THRESHOLD)
- **Output Format**: "X.XV" (e.g., "3.3V") or "Absent" if no voltage detected

The firmware uses this to prevent powering the target when external power is already present:
- If VTref > 0.5V when attempting to enable target power, the operation is cancelled
- GDB command output will show "Target already powered (X.XV)"

## Hardware Implementation

### Pin Assignment

**VTref Input: PB0** (Blackpill header pin)

### Connection Options

#### Option 1: Direct Connection (VTref ≤ 3.3V only)

For targets that are always 3.3V or lower:

```
Target VTref ----[1kΩ]---- PB0 (Blackpill)
                             |
                          [Optional: 100nF to GND for filtering]
```

The 1kΩ resistor provides current limiting protection for the ADC input.

#### Option 2: Voltage Divider (VTref up to 5V)

For targets that might be 5V:

```
Target VTref ----[10kΩ]----+---- PB0 (Blackpill)
                           |
                        [10kΩ]
                           |
                          GND
```

This divides the voltage by 2, allowing safe sensing of 5V targets (appears as 2.5V to ADC).

**Note**: Adjust the conversion formula in `platform_target_voltage_sense()` if using a voltage divider:
- Current formula assumes 1:1 (direct connection): `(val * 33U) / 4095U`
- For 2:1 divider (5V max): Change to `(val * 66U) / 4095U`

#### Option 3: Protection Circuit (Recommended for unknown targets)

For maximum protection with unknown target voltages:

```
Target VTref ----[10kΩ]----+----[BAT54S Schottky]---- +3.3V
                           |
                        [10kΩ]---- PB0 (Blackpill)
                           |
                        [10kΩ]
                           |
                          GND
```

This provides:
- Voltage division (3:1 = up to 9.9V input)
- Clamping diode protection to 3.3V rail
- Current limiting through resistors

### Important Notes

1. **STM32F4 ADC Input**: Maximum safe voltage is 3.3V (Vdd). Never exceed this on PB0.

2. **Pin Availability**: PB0 and PB1 are the only ADC-capable pins available on Blackpill headers that aren't already allocated for other functions.

3. **Build Requirement**: VTref sensing is only compiled when building with `SHIELD=1`:
   ```bash
   meson setup build --cross-file=cross-file/blackpill-f401cc.ini -Dshield=1
   ninja -C build
   ```

4. **Current Implementation**: Assumes direct connection or 1:1 voltage divider for 3.3V max. Modify the conversion formula if using different divider ratios.

5. **Simple Presence/Absence**: The implementation is optimized for detecting whether target power is present, not for precision voltage measurement. The 0.5V threshold provides reliable detection while avoiding false triggers from noise.

## Testing

To verify VTref sensing is working:

1. Build firmware with `SHIELD=1`
2. Connect VTref signal to PB0 with appropriate protection circuit
3. Connect to target via GDB
4. Use the monitor command to check voltage (if available)
5. Attempt to enable target power when target is already powered - should see "Target already powered (X.XV)" message

## Future Enhancements

Potential improvements for different use cases:

- Add calibration using internal VREFINT (ADC Channel 17) for better accuracy
- Support configurable voltage divider ratios via build options
- Add voltage monitoring/logging capabilities
- Implement over-voltage warning thresholds
