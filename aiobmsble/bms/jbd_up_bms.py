"""Module to support JBD UP smart BMS.

Newer JBD rack BMSs (e.g. UP16S) speak a MODBUS-like protocol using function
code 0x78 instead of the classic 0xDD frames handled by `jbd_bms`. The 0x1000
register block carries considerably more telemetry than the legacy 0x03 reply,
notably state of health, explicit MOSFET/ambient temperatures, operating limits
and auxiliary switch states.

Project: aiobmsble, https://pypi.org/p/aiobmsble/
License: Apache-2.0, http://www.apache.org/licenses/
"""

from functools import lru_cache
from typing import Final

from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.uuids import normalize_uuid_str

from aiobmsble import BMSConfig, BMSDp, BMSInfo, BMSSample, MatcherPattern, TempSensor
from aiobmsble._boundedbytearr import BoundedByteArray
from aiobmsble.basebms import BaseBMS, b2str, crc_modbus


class BMS(BaseBMS):
    """JBD UP smart BMS class implementation."""

    INFO: BMSInfo = {
        "default_manufacturer": "Jiabaida",
        "default_model": "UP smart BMS",
    }
    _HEAD_LEN: Final[int] = 8  # address, function, start, end (2 byte each), length
    _CRC_LEN: Final[int] = 2  # trailing CRC-16/MODBUS
    _ADDR: Final[int] = 0x01  # pack address, wildcard (0x00) is not relayed via BLE
    _FCT_READ: Final[int] = 0x78  # read register block
    _STATUS: Final[tuple[int, int]] = (0x1000, 0x10A0)  # pack status block
    _IDENT: Final[tuple[int, int]] = (0x1C00, 0x1C86)  # basic config/identity block
    _CELL_POS: Final[int] = 66  # position of the cell count field
    _MIN_LEN: Final[int] = _CELL_POS + 2  # all fixed position fields incl. cell count
    _CURR_BIAS: Final[int] = 300000  # current is transmitted with a fixed bias
    _TEMP_OFFS: Final[int] = 500  # temperatures are biased by 50.0 °C
    _MAX_CELL_COUNT: Final[int] = 32  # maximum number of cells supported
    _STR_LEN: Final[int] = 30  # NUL padded ASCII field length in identity block
    _FIELDS: Final[tuple[BMSDp, ...]] = (
        BMSDp("voltage", 0, 2, False, lambda x: x / 100),
        BMSDp("current", 4, 4, False, lambda x: (x - BMS._CURR_BIAS) / 100),
        BMSDp("battery_level", 8, 2, False, lambda x: x / 100),
        BMSDp("cycle_charge", 10, 2, False, lambda x: x / 100),
        BMSDp("design_capacity", 12, 2, False, lambda x: round(x / 100)),
        BMSDp("rated_capacity", 14, 2, False, lambda x: round(x / 100)),
        BMSDp("battery_health", 22, 2, False),
        BMSDp("dischrg_mosfet", 32, 2, False, lambda x: bool(x & 0x1)),
        BMSDp("chrg_mosfet", 32, 2, False, lambda x: bool(x & 0x2)),
        BMSDp("precharge_mosfet", 32, 2, False, lambda x: bool(x & 0x4)),
        BMSDp("heater", 32, 2, False, lambda x: bool(x & 0x8)),
        BMSDp("fan", 32, 2, False, lambda x: bool(x & 0x10)),
        BMSDp("cycles", 36, 2, False),
        BMSDp("chrg_voltage_limit", 58, 2, False, lambda x: x / 10),
        BMSDp("chrg_current_limit", 60, 2, False, lambda x: x / 10),
        BMSDp("dischrg_voltage_limit", 62, 2, False, lambda x: x / 10),
        BMSDp("dischrg_current_limit", 64, 2, False, lambda x: x / 10),
        BMSDp("cell_count", _CELL_POS, 2, False, lambda x: min(x, BMS._MAX_CELL_COUNT)),
    )

    def __init__(
        self,
        ble_device: BLEDevice,
        config: BMSConfig | None = None,
        logger_name: str = "",
    ) -> None:
        """Initialize private BMS members."""
        super().__init__(ble_device, config, logger_name)
        self._exp_block: int = 0x0000
        self._msg: bytes = b""

    @staticmethod
    def matcher_dict_list() -> list[MatcherPattern]:
        """Provide BluetoothMatcher definition."""
        return [
            MatcherPattern(
                oui="AA:C2:37",  # ECO-WORTHY 3U rack packs, e.g. ECO-LFP4850-3U
                service_uuid=BMS.uuid_services()[0],
                connectable=True,
            )
        ]

    @staticmethod
    def uuid_services() -> tuple[str, ...]:
        """Return list of 128-bit UUIDs of services required by BMS."""
        return (normalize_uuid_str("ff00"),)

    @staticmethod
    def uuid_rx() -> str:
        """Return 16-bit UUID of characteristic that provides notification/read property."""
        return "ff01"

    @staticmethod
    def uuid_tx() -> str:
        """Return 16-bit UUID of characteristic that provides write property."""
        return "ff02"

    @staticmethod
    @lru_cache(maxsize=8)
    def _cmd(block: tuple[int, int]) -> bytes:
        """Assemble a JBD UP read command for a register block."""
        frame: Final[bytes] = (
            BMS._ADDR.to_bytes(1)
            + BMS._FCT_READ.to_bytes(1)
            + block[0].to_bytes(2, "big")
            + block[1].to_bytes(2, "big")
            + (0).to_bytes(2, "big")  # a read request carries no payload
        )
        return frame + crc_modbus(frame).to_bytes(2, "little")

    @staticmethod
    def _frame_len(frame: bytes | bytearray | BoundedByteArray) -> int:
        """Return the total length a (partial) response announces."""
        return BMS._HEAD_LEN + int.from_bytes(frame[6:8], "big") + BMS._CRC_LEN

    def _notification_handler(
        self, _sender: BleakGATTCharacteristic, data: bytearray
    ) -> None:
        """Handle the RX characteristics notify event (new data arrives)."""
        head: Final[bytes] = self._exp_head()
        if data.startswith(head) and (
            len(self._frame) < BMS._HEAD_LEN
            or len(self._frame) >= BMS._frame_len(self._frame)
        ):
            # a reply start supersedes leftovers, unless a reply is still being
            # assembled (a payload chunk may coincidentally look like a start)
            self._frame.clear()

        if len(self._frame) + len(data) > self._frame.maxlen:
            self._log.debug("frame buffer overflow, discarding %s", self._frame)
            self._frame.clear()  # cannot be a valid reply, keep the incoming chunk

        self._frame.extend(data)
        self._log.debug(
            "RX BLE data (%s): %s", "start" if data == self._frame else "cnt.", data
        )

        if len(self._frame) < BMS._HEAD_LEN:
            return  # length field not received yet

        if bytes(self._frame[: len(head)]) != head:
            self._log.debug(
                "unexpected response (block 0x%X)",
                int.from_bytes(self._frame[2:4], "big"),
            )
            self._frame.clear()  # cannot become the awaited reply, discard
            return

        if BMS._frame_len(self._frame) > self._frame.maxlen:
            self._log.debug("implausible frame length: %s", self._frame)
            self._frame.clear()
            return

        if len(self._frame) < BMS._frame_len(self._frame):
            return  # response is not complete yet

        if len(self._frame) != BMS._frame_len(self._frame):
            self._log.debug("wrong data length (%i): %s", len(self._frame), self._frame)
            self._frame.clear()
            return

        if not self._check_integrity(
            self._frame,
            crc_modbus,
            slice(None, -BMS._CRC_LEN),
            slice(-BMS._CRC_LEN, None),
            "little",
        ):
            self._frame.clear()
            return

        self._msg = bytes(self._frame)
        self._msg_event.set()

    def _exp_head(self) -> bytes:
        """Return the header a reply to the pending request has to start with."""
        return (
            BMS._ADDR.to_bytes(1)
            + BMS._FCT_READ.to_bytes(1)
            + self._exp_block.to_bytes(2, "big")
        )

    async def _await_block(self, block: tuple[int, int]) -> bytes:
        """Request a register block and return its payload."""
        self._exp_block = block[0]
        self._frame.clear()  # never let a stale partial reply block this request
        try:
            await self._await_msg(BMS._cmd(block))
        finally:
            self._exp_block = 0x0000
        return self._msg[BMS._HEAD_LEN : -BMS._CRC_LEN]

    async def _fetch_device_info(self) -> BMSInfo:
        """Fetch the device information via BLE."""
        result: BMSInfo = {}
        status: Final[bytes] = await self._await_block(BMS._STATUS)
        if len(status) >= BMS._MIN_LEN:
            cells: Final[int] = int.from_bytes(
                status[BMS._CELL_POS : BMS._CELL_POS + 2], "big"
            )
            temps: Final[int] = int.from_bytes(
                status[68 + 2 * cells : 70 + 2 * cells], "big"
            )
            # cell voltages, temperature count/values, balance mask and reserved word
            pos: Final[int] = 70 + 2 * cells + 2 * temps + 4
            if pos + 2 <= len(status):
                result["sw_version"] = f"{status[pos]}.{status[pos + 1]}"

        try:
            ident: Final[bytes] = await self._await_block(BMS._IDENT)
        except TimeoutError:
            return result

        if serial := b2str(ident[16 : 16 + BMS._STR_LEN]):
            result["serial_number"] = serial
        if model := b2str(ident[52 : 52 + BMS._STR_LEN]):
            result["model_id"] = model

        return result

    async def _async_update(self) -> BMSSample:
        """Update battery status information."""
        data: Final[bytes] = await self._await_block(BMS._STATUS)
        if len(data) < BMS._MIN_LEN:
            raise ValueError("BMS data incomplete.")

        result: BMSSample = BMS._decode_data(BMS._FIELDS, data)
        cells: Final[int] = result.get("cell_count", 0)
        result["cell_voltages"] = BMS._cell_voltages(data, cells=cells, start=68)

        temp_pos: Final[int] = 70 + 2 * cells
        temp_cnt: Final[int] = int.from_bytes(data[temp_pos - 2 : temp_pos], "big")
        result["temp_values"] = BMS._temp_values(
            data,
            start=16,
            values=2,
            signed=False,
            offset=BMS._TEMP_OFFS,
            divider=10,
            types=(TempSensor.T.MOSFET, TempSensor.T.AMBIENT),
        ) + BMS._temp_values(
            data,
            start=temp_pos,
            values=temp_cnt,
            signed=False,
            offset=BMS._TEMP_OFFS,
            divider=10,
            types=(TempSensor.T.CELL,) * temp_cnt,
        )
        result["temp_sensors"] = len(result["temp_values"])

        bal_pos: Final[int] = temp_pos + 2 * temp_cnt
        if bal_pos + 2 <= len(data):
            result["balancer"] = int.from_bytes(data[bal_pos : bal_pos + 2], "big")

        # protection bitmask in the lower, alarm/error bitmask in the upper word
        result["problem_code"] = int.from_bytes(data[24:28], "big") | (
            int.from_bytes(data[28:32], "big") << 32
        )

        return result
