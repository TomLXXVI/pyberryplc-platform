import serial
import time

def calculate_crc(data: list[int]) -> int:
    crc = 0
    for b in data:
        for _ in range(8):
            # noinspection PyRedundantParentheses
            if ((crc >> 7) ^ (b & 0x01)):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            b >>= 1
    return crc


def read_register(uart, register: int, slave_address: int = 0x00) -> None:
    # noinspection PyListCreation
    request = [
        0x05,               # Sync
        slave_address,
        register & 0x7F     # Clear write bit
    ]
    request.append(calculate_crc(request))

    print("🟦 Transmit datagram:")
    for i, b in enumerate(request):
        print(f"  Byte {i}: 0x{b:02X}")

    uart.write(bytes(request))
    uart.flush()

    print("\n⏳ Waiting for response...")
    response = uart.read(12)
    response = response[4:]
    
    if len(response) < 8:
        print(f"❌ Incomplete response received: {response.hex()}")
        return

    print("\n🟩 Received datagram:")
    for i, b in enumerate(response):
        print(f"  Byte {i}: 0x{b:02X}")

    if response[0] != 0x05:
        print("❌ Invalid sync byte in response.")
        return
    
    master_address = 0xFF
    header_ok = response[1] == master_address and response[2] == (register & 0x7F)
    crc_ok = calculate_crc(list(response[:7])) == response[7]

    print(f"\n✅ Header OK? {header_ok}")
    print(f"✅ CRC check: calculated 0x{calculate_crc(list(response[:7])):02X}, received 0x{response[7]:02X} → {'OK' if crc_ok else '❌ MISMATCH'}")

    if header_ok and crc_ok:
        value = (
            (response[3] << 24) |
            (response[4] << 16) |
            (response[5] << 8) |
            response[6]
        )
        print(f"\n📦 Register 0x{register:02X} value: 0x{value:08X} (bin: {value:032b})")
    else:
        print("⚠️  Response header or CRC is invalid.")


if __name__ == "__main__":
    uart = serial.Serial(port="/dev/ttyAMA0", baudrate=115200, timeout=0.5)
    try:
        for _ in range(2):
            read_register(uart, register=0x6C)
            time.sleep(0.1)
    finally:
        uart.close()
