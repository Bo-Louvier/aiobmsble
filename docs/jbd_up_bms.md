# JBD UP BMS

Newer JBD rack BMSs (marketed as "UP", e.g. UP16S) do not use the classic
`0xDD` frames handled by [`jbd_bms`](jbd_bms.md). They speak a MODBUS-like
protocol with the vendor function code `0x78` (read register block) over the
same GATT profile (service `0xFF00`, notify `0xFF01`, write `0xFF02`).

The protocol was reverse engineered over RS485 and verified to be relayed
unchanged by the pack's BLE dongle on an ECO-WORTHY ECO-LFP4850-3U
(BMS model `JBD48100000`, firmware 12.4).

## Frame format

| Field | Size | Notes |
|---|---|---|
| Address | 1 | `0x01`; the wildcard `0x00` is **not** answered over BLE |
| Function | 1 | `0x78` read register block |
| Start address | 2 | big endian |
| End address | 2 | big endian |
| Data length | 2 | big endian; `0` in a request |
| Payload | *n* | response only, `n` = data length |
| CRC | 2 | CRC-16/MODBUS, **little** endian, over all preceding bytes |

Request for the pack status block:

```
01 78 10 00 10 A0 00 00 7F B2
└addr └fct └0x1000 └0x10A0 └len 0 └CRC (LE)
```

Responses arrive fragmented across BLE notifications and are reassembled until
`8 + data length + 2` bytes are present.

## Pack status block (`0x1000`–`0x10A0`)

Offsets are byte positions within the response payload. On a UP16S the payload
is 158 bytes.

| Offset | Field | Encoding | `BMSSample` key |
|---:|---|---|---|
| 0 | total voltage | u16 ×0.01 V | `voltage` |
| 4 | current | u32, (raw − 300000) ×0.01 A | `current` |
| 8 | state of charge | u16 ×0.01 % | `battery_level` |
| 10 | capacity remaining | u16 ×0.01 Ah | `cycle_charge` |
| 12 | nominal capacity | u16 ×0.01 Ah | `design_capacity` |
| 14 | rated capacity | u16 ×0.01 Ah | `rated_capacity` |
| 16 | MOSFET temperature | u16, (raw − 500) ×0.1 °C | `temp_values[0]` |
| 18 | ambient temperature | u16, (raw − 500) ×0.1 °C | `temp_values[1]` |
| 20 | operation status | u16, 0 idle, 1 charge, 2 discharge | *not used* |
| 22 | state of health | u16 % | `battery_health` |
| 24 | protection bitmask | u32 | `problem_code` (low word) |
| 28 | error/alarm bitmask | u32 | `problem_code` (high word) |
| 32 | MOSFET status | u16 bitfield | see below |
| 36 | charging cycles | u16 | `cycles` |
| 38/40 | max cell index / voltage | u16, ×0.001 V | *derived* |
| 42/44 | min cell index / voltage | u16, ×0.001 V | *derived* |
| 46 | average cell voltage | u16 ×0.001 V | *derived* |
| 48–56 | max/min/avg temperature | u16 index and values | *derived* |
| 58 | charge voltage limit | u16 ×0.1 V | `chrg_voltage_limit` |
| 60 | charge current limit | u16 ×0.1 A | `chrg_current_limit` |
| 62 | discharge voltage limit | u16 ×0.1 V | `dischrg_voltage_limit` |
| 64 | discharge current limit | u16 ×0.1 A | `dischrg_current_limit` |
| 66 | cell count *N* | u16 | `cell_count` |
| 68 | cell voltages | *N* × u16 ×0.001 V | `cell_voltages` |
| 68+2*N* | temperature sensor count *M* | u16 | |
| 70+2*N* | temperatures | *M* × u16, (raw − 500) ×0.1 °C | `temp_values[2:]` |
| 70+2*N*+2*M* | balance status | u16 bitmask, one bit per cell | `balancer` |
| +2 | reserved | u16 | |
| +4 | firmware version | u16, high byte.low byte | `sw_version` |
| +6 | device model | 30 B NUL padded ASCII | |

### MOSFET status bits (offset 32)

| Bit | Meaning | `BMSSample` key |
|---:|---|---|
| 0 | discharge MOSFET | `dischrg_mosfet` |
| 1 | charge MOSFET | `chrg_mosfet` |
| 2 | precharge MOSFET | `precharge_mosfet` |
| 3 | heater | `heater` |
| 4 | fan | `fan` |

### Temperature sensors

`temp_values` reports the explicit MOSFET and ambient sensors first (typed
`MOSFET` and `AMBIENT`), followed by the *M* cell sensors from the trailing
array (typed `CELL`). On a UP16S that is six sensors in total, matching the six
NTC values the legacy `0x03` frame reports in the order
`[MOSFET, ambient, sensor 1..4]`.

### Capacity fields

The block reports three capacities. The BMS references its own state of charge
to the **nominal** capacity, not to the nameplate rating: on the reference pack
52.19 Ah remaining of 53.97 Ah nominal is reported as 96.71 %, while the
nameplate rating is 50.00 Ah. `design_capacity` therefore maps to the nominal
capacity so that it stays consistent with `battery_level` and `cycle_charge`;
the nameplate value is exposed separately as `rated_capacity`.

Note that state of health is not clamped by the BMS and can exceed 100 % (the
reference pack reports 107 %).

### Problem code

`problem_code` combines both bitmasks: the protection mask (offset 24) in the
lower 32 bits and the error/alarm mask (offset 28) in the upper 32 bits.

Protection mask bits: 0 cell overvoltage, 1 cell undervoltage, 2 pack
overvoltage, 3 pack undervoltage, 4 charge overcurrent 1, 5 charge overcurrent 2,
6 discharge overcurrent 1, 7 discharge overcurrent 2, 8 charge over-temperature,
9 charge under-temperature, 10 discharge over-temperature, 11 discharge
under-temperature, 12 MOSFET over-temperature, 13 ambient over-temperature,
14 ambient under-temperature, 15 voltage difference, 16 temperature difference,
17 SOC low, 18 short circuit, 19 monomer offline, 20 temperature drop, 21 charge
MOSFET fault, 22 discharge MOSFET fault, 23 current limiting fault, 24 aerosol
fault, 25 full charge protection, 26 abnormal analog front end communication.

Error/alarm mask bits: 0 cell overvoltage, 1 cell undervoltage, 2 pack
overvoltage, 3 pack undervoltage, 4 charge overcurrent, 5 discharge overcurrent,
6 charge over-temperature, 7 charge under-temperature, 8 discharge
over-temperature, 9 discharge overcurrent, 10 MOSFET over-temperature,
11 ambient over-temperature, 12 ambient under-temperature, 13 voltage difference
too large, 14 temperature difference too large, 15 SOC too low, 16 EEPROM fault,
17 real time clock abnormal.

## Identity block (`0x1C00`–`0x1C86`)

Read once per connection for device information; a timeout is tolerated.

| Offset | Size | Field |
|---:|---:|---|
| 16 | 30 | serial number, NUL padded ASCII (26 characters, `UP16S…`) |
| 52 | 30 | device model, NUL padded ASCII (e.g. `JBD48100000`) |
| 88 | 30 | device name, NUL padded ASCII (the BLE local name, e.g. `ECO-LFP4850-3U-xxxxxx` with the last three MAC bytes as suffix) |

## Device detection

The known packs of this family advertise service `0xFF00` with OUI `AA:C2:37`
and a local name of the form `ECO-LFP*`. That OUI was previously matched by
`jbd_bms` (added for BMS_BLE-HA issue #284, which reported an
`ECO-LFP48100-3U` rack pack) and has been moved to this plugin, because
`MatcherPattern` entries are combined with AND and offer no negation — the OUI
cannot be shared between both plugins without making advertisements ambiguous.

If a device with this OUI turns out to speak only the legacy `0xDD` protocol,
the matcher has to be narrowed on both sides (e.g. by `local_name`) rather than
duplicated.

## Not exposed

The following values are read but have no `BMSSample` representation:

- operation status (offset 20) — redundant with the sign of `current`, which
  already drives the derived `battery_charging`;
- the max/min/average cell voltage and temperature summaries (offsets 38–56) —
  derivable from `cell_voltages` and `temp_values`.

Write support (function code `0x79`, including the unlock handshake and the
protection register map at `0x1800`–`0x18CE`) is intentionally not implemented;
this library is read-only.
