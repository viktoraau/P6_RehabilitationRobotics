import ctypes
import errno
import fcntl
import glob
import os
from typing import List, Optional, Tuple


GPIO_MAX_NAME_SIZE = 32
GPIOHANDLES_MAX = 64

GPIOHANDLE_REQUEST_OUTPUT = 1 << 1

_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14

_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS

_IOC_WRITE = 1
_IOC_READ = 2


def _ioc(direction: int, ioctl_type: int, number: int, size: int) -> int:
    return (
        (direction << _IOC_DIRSHIFT)
        | (ioctl_type << _IOC_TYPESHIFT)
        | (number << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def _iowr(ioctl_type: int, number: int, struct_type: type) -> int:
    return _ioc(_IOC_READ | _IOC_WRITE, ioctl_type, number, ctypes.sizeof(struct_type))


class GpioError(RuntimeError):
    pass


class _GpioHandleRequest(ctypes.Structure):
    _fields_ = [
        ("lineoffsets", ctypes.c_uint32 * GPIOHANDLES_MAX),
        ("flags", ctypes.c_uint32),
        ("default_values", ctypes.c_uint8 * GPIOHANDLES_MAX),
        ("consumer_label", ctypes.c_char * GPIO_MAX_NAME_SIZE),
        ("lines", ctypes.c_uint32),
        ("fd", ctypes.c_int),
    ]


class _GpioHandleData(ctypes.Structure):
    _fields_ = [("values", ctypes.c_uint8 * GPIOHANDLES_MAX)]


class _GpioLineInfo(ctypes.Structure):
    _fields_ = [
        ("line_offset", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("name", ctypes.c_char * GPIO_MAX_NAME_SIZE),
        ("consumer", ctypes.c_char * GPIO_MAX_NAME_SIZE),
    ]


GPIO_GET_LINEINFO_IOCTL = _iowr(0xB4, 0x02, _GpioLineInfo)
GPIO_GET_LINEHANDLE_IOCTL = _iowr(0xB4, 0x03, _GpioHandleRequest)
GPIOHANDLE_SET_LINE_VALUES_IOCTL = _iowr(0xB4, 0x09, _GpioHandleData)


def _read_text(path: str) -> str:
    with open(path, encoding="utf-8") as file:
        return file.read().strip()


def _available_gpiochips() -> List[str]:
    descriptions = []

    for sysfs_chip in sorted(glob.glob("/sys/class/gpio/gpiochip*")):
        label_path = os.path.join(sysfs_chip, "label")
        label = _read_text(label_path) if os.path.exists(label_path) else "unknown"
        char_devices = []

        for candidate in sorted(glob.glob(os.path.join(sysfs_chip, "device", "gpiochip*"))):
            dev_path = os.path.join("/dev", os.path.basename(candidate))
            if os.path.exists(dev_path):
                char_devices.append(dev_path)

        if not char_devices:
            fallback = os.path.join("/dev", os.path.basename(sysfs_chip))
            if os.path.exists(fallback):
                char_devices.append(fallback)

        descriptions.append(f"{label}: {', '.join(char_devices) if char_devices else 'no device'}")

    return descriptions


def resolve_gpiochip_path(chip_path: str, chip_label: str) -> str:
    if chip_path:
        if not os.path.exists(chip_path):
            raise GpioError(f"Configured gpio chip path does not exist: {chip_path}")
        return chip_path

    if not chip_label:
        raise GpioError("Either gpio_chip_path or gpio_chip_label must be provided")

    for sysfs_chip in sorted(glob.glob("/sys/class/gpio/gpiochip*")):
        label_path = os.path.join(sysfs_chip, "label")
        if not os.path.exists(label_path):
            continue

        if _read_text(label_path) != chip_label:
            continue

        for candidate in sorted(glob.glob(os.path.join(sysfs_chip, "device", "gpiochip*"))):
            dev_path = os.path.join("/dev", os.path.basename(candidate))
            if os.path.exists(dev_path):
                return dev_path

        fallback = os.path.join("/dev", os.path.basename(sysfs_chip))
        if os.path.exists(fallback):
            return fallback

    available = "; ".join(_available_gpiochips())
    raise GpioError(
        f"Could not resolve gpio chip label '{chip_label}'. Available gpio chips: {available}"
    )


def get_gpio_line_info(chip_path: str, line_offset: int) -> Optional[Tuple[str, str, int]]:
    try:
        chip_fd = os.open(chip_path, os.O_RDONLY)
    except OSError:
        return None

    try:
        info = _GpioLineInfo()
        info.line_offset = line_offset
        fcntl.ioctl(chip_fd, GPIO_GET_LINEINFO_IOCTL, info)
        return (
            info.name.decode("utf-8", errors="ignore").rstrip("\x00"),
            info.consumer.decode("utf-8", errors="ignore").rstrip("\x00"),
            int(info.flags),
        )
    except OSError:
        return None
    finally:
        os.close(chip_fd)


class LinuxCharGpioBackend:
    def __init__(
        self,
        chip_path: str,
        chip_label: str,
        line_offset: int,
        consumer_label: str,
        active_high: bool = True,
    ) -> None:
        if line_offset < 0:
            raise GpioError("GPIO line offset must be non-negative")

        self.chip_path = resolve_gpiochip_path(chip_path, chip_label)
        self.line_offset = int(line_offset)
        self.consumer_label = consumer_label
        self.active_high = bool(active_high)
        self._handle_fd = self._request_output_handle(initial_enabled=False)

    def _logical_to_physical(self, enabled: bool) -> int:
        if self.active_high:
            return 1 if enabled else 0
        return 0 if enabled else 1

    def _request_output_handle(self, initial_enabled: bool) -> int:
        request = _GpioHandleRequest()
        request.lineoffsets[0] = self.line_offset
        request.flags = GPIOHANDLE_REQUEST_OUTPUT
        request.default_values[0] = self._logical_to_physical(initial_enabled)
        request.consumer_label = self.consumer_label.encode("utf-8")[: GPIO_MAX_NAME_SIZE - 1]
        request.lines = 1

        try:
            chip_fd = os.open(self.chip_path, os.O_RDONLY)
        except OSError as exc:
            raise GpioError(f"Could not open {self.chip_path}: {exc}") from exc

        try:
            fcntl.ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, request)
        except OSError as exc:
            details = ""
            if exc.errno == errno.EBUSY:
                line_info = get_gpio_line_info(self.chip_path, self.line_offset)
                if line_info is not None:
                    line_name, consumer, _ = line_info
                    consumer_text = consumer or "unknown consumer"
                    details = (
                        f" Line {self.line_offset}"
                        f"{f' ({line_name})' if line_name else ''} is already held by "
                        f"'{consumer_text}'."
                    )
            raise GpioError(
                f"Could not request GPIO line {self.line_offset} from {self.chip_path}: {exc}."
                f"{details}"
            ) from exc
        finally:
            os.close(chip_fd)

        if request.fd < 0:
            raise GpioError(
                f"Kernel returned an invalid GPIO handle for line {self.line_offset}"
            )

        return request.fd

    def set_output(self, enabled: bool) -> None:
        values = _GpioHandleData()
        values.values[0] = self._logical_to_physical(enabled)

        try:
            fcntl.ioctl(self._handle_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, values)
        except OSError as exc:
            raise GpioError(
                f"Failed to drive GPIO line {self.line_offset} on {self.chip_path}: {exc}"
            ) from exc

    def close(self) -> None:
        if self._handle_fd is None:
            return

        try:
            self.set_output(False)
        finally:
            os.close(self._handle_fd)
            self._handle_fd = None


class MockGpioBackend:
    def __init__(self) -> None:
        self.chip_path = "mock"
        self.line_offset = -1
        self.last_enabled = False

    def set_output(self, enabled: bool) -> None:
        self.last_enabled = bool(enabled)

    def close(self) -> None:
        self.last_enabled = False
