"""Test the JBD UP BMS implementation."""

import asyncio
from collections.abc import Buffer, Callable, Iterable
from typing import Any, Final
from uuid import UUID

from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.uuids import normalize_uuid_str
import pytest

from aiobmsble import BMSConfig, BMSSample, TempSensor as TS
from aiobmsble.basebms import crc_modbus
from aiobmsble.bms.jbd_up_bms import BMS
from tests.bluetooth import generate_ble_device
from tests.conftest import MockBleakClient
from tests.test_basebms import BMSBasicTests

BT_FRAME_SIZE: Final[int] = 20

# recorded from an ECO-WORTHY ECO-LFP4850-3U (JBD UP16S, fw 12.4), read-only;
# serial number and device name in the identity block are masked (CRC recomputed)
_STATUS_RSP: Final[bytes] = (
    b"\x01\x78\x10\x00\x10\xa0\x00\x9e\x14\xcb\x00\x00\x00\x04\x93\xe0\x25\xc7\x14\x63\x15\x15\x13\x88"
    b"\x03\x0c\x03\x1f\x00\x00\x00\x6b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x04\x00\x57\x00\x02"
    b"\x0d\x03\x00\x06\x0c\xfe\x0c\xff\x00\x03\x03\x14\x00\x02\x03\x08\x03\x0c\x02\x48\x03\xe8\x01\xc0"
    b"\x03\xe8\x00\x10\x0c\xff\x0d\x03\x0c\xff\x0c\xff\x0c\xff\x0c\xfe\x0c\xff\x0c\xff\x0d\x00\x0c\xff"
    b"\x0c\xff\x0c\xff\x0c\xff\x0c\xff\x0c\xff\x0c\xff\x00\x04\x03\x09\x03\x08\x03\x14\x03\x0b\x00\x00"
    b"\x00\x00\x0c\x04\x4a\x42\x44\x34\x38\x31\x30\x30\x30\x30\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x5a\xa6\x00\x00\x00\x00\x00\x00\xfc\xce"
)
_IDENT_RSP: Final[bytes] = (
    b"\x01\x78\x1c\x00\x1c\x86\x00\x86\x00\x00\x00\x00\x0d\x02\x00\x0a\x01\xf4\x02\x58\x16\x30\x05\xdc"
    b"\x55\x50\x31\x36\x53\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30"
    b"\x30\x30\x00\x00\x00\x00\x07\xe9\x00\x04\x00\x18\x4a\x42\x44\x34\x38\x31\x30\x30\x30\x30\x30\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x07\xe9\x00\x01\x00\x01"
    b"\x45\x43\x4f\x2d\x4c\x46\x50\x34\x38\x35\x30\x2d\x33\x55\x2d\x30\x30\x30\x30\x30\x30\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x0c\x4e\x05\xa0\x00\x01\x00\x00\x00\x00\x00\x00\x00\x05\x07\x93"
)

_RESULT_DEFS: Final[BMSSample] = {
    "voltage": 53.23,
    "current": 0.0,
    "battery_level": 96.71,
    "cycle_charge": 52.19,
    "design_capacity": 54,
    "rated_capacity": 50,
    "battery_health": 107,
    "cycles": 87,
    "cell_count": 16,
    "cell_voltages": [
        3.327, 3.331, 3.327, 3.327, 3.327, 3.326, 3.327, 3.327,
        3.328, 3.327, 3.327, 3.327, 3.327, 3.327, 3.327, 3.327,
    ],  # fmt: skip
    "delta_voltage": 0.005,
    "temp_sensors": 6,
    "temp_values": [
        TS(28.0, TS.T.MOSFET),
        TS(29.9, TS.T.AMBIENT),
        TS(27.7, TS.T.CELL),
        TS(27.6, TS.T.CELL),
        TS(28.8, TS.T.CELL),
        TS(27.9, TS.T.CELL),
    ],
    "temperature": 28.317,
    "chrg_voltage_limit": 58.4,
    "chrg_current_limit": 100.0,
    "dischrg_voltage_limit": 44.8,
    "dischrg_current_limit": 100.0,
    "chrg_mosfet": True,
    "dischrg_mosfet": True,
    "precharge_mosfet": False,
    "heater": False,
    "fan": False,
    "balancer": 0,
    "problem_code": 0,
    "problem": False,
    "power": 0.0,
    "battery_charging": False,
    "cycle_capacity": 2778.074,
}


def _frame(payload: bytes, block: tuple[int, int] = BMS._STATUS) -> bytes:
    """Assemble a valid UP response frame around the given payload."""
    head: Final[bytes] = (
        b"\x01\x78"
        + block[0].to_bytes(2, "big")
        + block[1].to_bytes(2, "big")
        + len(payload).to_bytes(2, "big")
    )
    return head + payload + crc_modbus(head + payload).to_bytes(2, "little")


class TestBasicBMS(BMSBasicTests):
    """Test the basic BMS functionality."""

    bms_class = BMS


class MockJBDUPBleakClient(MockBleakClient):
    """Emulate a JBD UP BMS BleakClient."""

    RESP: dict[int, bytes] = {
        BMS._STATUS[0]: _STATUS_RSP,
        BMS._IDENT[0]: _IDENT_RSP,
    }

    _tasks: set[asyncio.Task[None]] = set()

    def __init__(
        self,
        address_or_ble_device: BLEDevice,
        disconnected_callback: Callable[[BleakClient], None] | None,
        services: Iterable[str] | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize MockBleakClient."""
        super().__init__(
            address_or_ble_device, disconnected_callback, services, **kwargs
        )
        self._services = ["ff01", "ff02"]

    def _response(
        self, char_specifier: BleakGATTCharacteristic | int | str | UUID, data: Buffer
    ) -> bytes:
        msg: Final[bytes] = bytes(data)
        if (
            not isinstance(char_specifier, str)
            or normalize_uuid_str(char_specifier) != normalize_uuid_str("ff02")
            or not msg.startswith(b"\x01\x78")
        ):
            return b""
        return self.RESP.get(int.from_bytes(msg[2:4], "big"), b"")

    async def _send_data(
        self, char_specifier: BleakGATTCharacteristic | int | str | UUID, data: Buffer
    ) -> None:
        assert (
            self._notify_callback
        ), "write to characteristics but notification not enabled"

        resp: Final[bytes] = self._response(char_specifier, data)
        for notify_data in [
            resp[i : i + BT_FRAME_SIZE] for i in range(0, len(resp), BT_FRAME_SIZE)
        ]:
            self._notify_callback("MockJBDUPBleakClient", bytearray(notify_data))
        await asyncio.sleep(0)

    async def write_gatt_char(
        self,
        char_specifier: BleakGATTCharacteristic | int | str | UUID,
        data: Buffer,
        response: bool | None = None,
    ) -> None:
        """Issue write command to GATT."""
        task: Final[asyncio.Task[None]] = asyncio.create_task(
            self._send_data(char_specifier, data), name="send_loop"
        )
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def disconnect(self) -> None:
        """Mock disconnect."""
        await asyncio.gather(*self._tasks)
        await super().disconnect()


async def test_update(patch_bleak_client, keep_alive_fixture: bool) -> None:
    """Test JBD UP BMS data update."""
    patch_bleak_client(MockJBDUPBleakClient)

    bms = BMS(generate_ble_device(), BMSConfig(keep_alive_fixture))

    assert await bms.async_update() == _RESULT_DEFS

    # query again to check already connected state
    await bms.async_update()
    assert bms.is_connected is keep_alive_fixture

    await bms.disconnect()


async def test_device_info(patch_bleak_client) -> None:
    """Test that the BMS returns the device information from the identity block."""
    patch_bleak_client(MockJBDUPBleakClient)

    bms = BMS(generate_ble_device())
    assert await bms.device_info() == {
        "sw_version": "12.4",
        "serial_number": "UP16S000000000000000000000",
        "model_id": "JBD48100000",
    }
    await bms.disconnect()


async def test_no_ident_block(patch_bleak_client, patch_bms_timeout) -> None:
    """Test that device info degrades gracefully without the identity block."""
    patch_bms_timeout("jbd_up_bms")

    class MockNoIdentClient(MockJBDUPBleakClient):
        """Emulate a BMS that does not answer the identity block read."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: _STATUS_RSP}

    patch_bleak_client(MockNoIdentClient)

    bms = BMS(generate_ble_device())
    assert await bms.device_info() == {"sw_version": "12.4"}
    await bms.disconnect()


async def test_empty_ident_strings(patch_bleak_client) -> None:
    """Test that blank identity strings are not reported."""

    class MockBlankIdentClient(MockJBDUPBleakClient):
        """Emulate a BMS with a NUL filled identity block."""

        RESP: dict[int, bytes] = {
            BMS._STATUS[0]: _STATUS_RSP,
            BMS._IDENT[0]: _frame(bytes(134), BMS._IDENT),
        }

    patch_bleak_client(MockBlankIdentClient)

    bms = BMS(generate_ble_device())
    assert await bms.device_info() == {"sw_version": "12.4"}
    await bms.disconnect()


async def test_truncated_status(patch_bleak_client, patch_bms_timeout) -> None:
    """Test that a status block without firmware/balancer data is handled."""
    patch_bms_timeout("jbd_up_bms")

    # keep cell count and temperature count, drop everything behind the readings
    payload: Final[bytes] = _STATUS_RSP[8:-2][:110]

    class MockShortStatusClient(MockJBDUPBleakClient):
        """Emulate a BMS that reports a shortened status block."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: _frame(payload)}

    patch_bleak_client(MockShortStatusClient)

    bms = BMS(generate_ble_device())
    result: Final[BMSSample] = await bms.async_update()
    assert "balancer" not in result
    assert result["cell_count"] == 16
    assert await bms.device_info() == {}
    await bms.disconnect()


async def test_invalid_response(patch_bleak_client, patch_bms_timeout) -> None:
    """Test that an incomplete status block is rejected."""
    patch_bms_timeout("jbd_up_bms")

    class MockTinyStatusClient(MockJBDUPBleakClient):
        """Emulate a BMS that reports a status block that is far too short."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: _frame(bytes(16))}

    patch_bleak_client(MockTinyStatusClient)

    bms = BMS(generate_ble_device())
    with pytest.raises(ValueError, match="BMS data incomplete."):
        await bms.async_update()
    assert await bms.device_info() == {}  # no firmware version without readings
    await bms.disconnect()


@pytest.mark.parametrize(
    "wrong_response",
    [
        _STATUS_RSP[:-1] + b"\x00",  # invalid CRC
        _frame(_STATUS_RSP[8:-2], (0x2000, 0x2050)),  # unrequested block
        _STATUS_RSP + b"\x00",  # oversized frame
        _STATUS_RSP[:4],  # frame shorter than the header
        _STATUS_RSP[:6] + b"\xff\xff" + _STATUS_RSP[8:],  # length exceeds buffer
    ],
    ids=["invalid_crc", "wrong_block", "oversized", "no_length", "huge_length"],
)
async def test_invalid_frame(
    patch_bleak_client, patch_bms_timeout, wrong_response: bytes
) -> None:
    """Test that malformed frames are discarded."""
    patch_bms_timeout("jbd_up_bms")

    class MockInvalidClient(MockJBDUPBleakClient):
        """Emulate a BMS that returns a malformed frame."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: wrong_response}

    patch_bleak_client(MockInvalidClient)

    bms = BMS(generate_ble_device())
    with pytest.raises(TimeoutError):
        await bms.async_update()
    await bms.disconnect()


async def test_stale_partial_frame(patch_bleak_client, patch_bms_timeout) -> None:
    """Test that a truncated reply left behind by a timeout does not block later requests."""
    patch_bms_timeout("jbd_up_bms")

    class MockTruncatedIdentClient(MockJBDUPBleakClient):
        """Emulate a BMS whose identity reply breaks off before the length field."""

        RESP: dict[int, bytes] = {
            BMS._STATUS[0]: _STATUS_RSP,
            BMS._IDENT[0]: _IDENT_RSP[: BMS._HEAD_LEN - 3],
        }

    patch_bleak_client(MockTruncatedIdentClient)

    bms = BMS(generate_ble_device())
    assert await bms.async_update() == _RESULT_DEFS
    # connection stays open, so the partial identity reply remains in the buffer
    assert await bms.device_info() == {"sw_version": "12.4"}
    assert await bms.async_update() == _RESULT_DEFS
    await bms.disconnect()


async def test_payload_looks_like_frame_start(patch_bleak_client) -> None:
    """Test that a payload chunk starting with the reply header does not drop the reply."""
    # design/rated capacity words equal to the expected header, right at a chunk boundary
    payload: Final[bytearray] = bytearray(_STATUS_RSP[8:-2])
    payload[12:16] = b"\x01\x78\x10\x00"
    assert BT_FRAME_SIZE == 20  # chunk 2 starts at payload offset 12

    class MockHeaderInPayloadClient(MockJBDUPBleakClient):
        """Emulate a BMS whose status payload contains the reply header bytes."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: _frame(bytes(payload))}

    patch_bleak_client(MockHeaderInPayloadClient)

    bms = BMS(generate_ble_device())
    assert await bms.async_update() == _RESULT_DEFS | {
        "design_capacity": 4,  # 0x0178 = 3.76 Ah
        "rated_capacity": 41,  # 0x1000 = 40.96 Ah
    }
    await bms.disconnect()


@pytest.mark.parametrize("partial_len", [7, 20], ids=["below_header", "with_header"])
async def test_partial_first_attempt(
    patch_bleak_client, patch_bms_timeout, partial_len: int
) -> None:
    """Test that a reply cut short on the first attempt does not block the retries."""
    patch_bms_timeout("jbd_up_bms")

    class MockPartialFirstClient(MockJBDUPBleakClient):
        """Emulate a BMS that truncates the very first status reply."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: _STATUS_RSP}
        _requests: int = 0

        def _response(
            self,
            char_specifier: BleakGATTCharacteristic | int | str | UUID,
            data: Buffer,
        ) -> bytes:
            resp: Final[bytes] = super()._response(char_specifier, data)
            MockPartialFirstClient._requests += 1
            return resp[:partial_len] if MockPartialFirstClient._requests == 1 else resp

    patch_bleak_client(MockPartialFirstClient)

    bms = BMS(generate_ble_device())
    assert await bms.async_update() == _RESULT_DEFS
    assert MockPartialFirstClient._requests > 1
    await bms.disconnect()


async def test_buffer_overflow_recovery(patch_bleak_client, patch_bms_timeout) -> None:
    """Test that a reply filling the buffer after a stale partial cannot overflow it."""
    patch_bms_timeout("jbd_up_bms")
    # largest reply the buffer can hold; a stale header in front of it would overflow
    big: Final[bytes] = _frame(bytes(1014))

    class MockOverflowClient(MockJBDUPBleakClient):
        """Emulate a BMS that first sends only a header, then a buffer sized reply."""

        RESP: dict[int, bytes] = {BMS._STATUS[0]: big}
        _requests: int = 0

        def _response(
            self,
            char_specifier: BleakGATTCharacteristic | int | str | UUID,
            data: Buffer,
        ) -> bytes:
            resp: Final[bytes] = super()._response(char_specifier, data)
            MockOverflowClient._requests += 1
            return resp[: BMS._HEAD_LEN] if MockOverflowClient._requests == 1 else resp

    patch_bleak_client(MockOverflowClient)

    bms = BMS(generate_ble_device())
    assert (await bms.async_update()).get("voltage") == 0
    assert MockOverflowClient._requests > 1
    await bms.disconnect()
