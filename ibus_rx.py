import ustruct

class IBus:
    def __init__(self, uart):
        self.uart = uart
        self.uart.init(baudrate=115200, bits=8, parity=None, stop=1)

    def read_raw(self):
        if not self.uart.any():
            return

        packet = self.uart.read(32)
        if len(packet)!=32:
            return

        # ibus packet format:
        # header (0x2040)
        # 13 channel (2 bytes each, little endian)
        # 2 bytes checksum (0xFFFF - all previous bytes)

        # packet bytes + checksum should equal 0xFFFF
        checksum = sum(b for b in ustruct.unpack("<30BH", packet))

        # unpack and verify data integrity
        data = ustruct.unpack("<BB14H", packet)
        if data[0] == 0x20 and data[1] == 0x40 and checksum == 0xFFFF:
            return data[2:-1]

def normalise(value):
    # midpoint 1500
    # range [988; 2012]
    return (value-1500) / 512.0