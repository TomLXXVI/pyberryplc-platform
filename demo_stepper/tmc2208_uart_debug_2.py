from pyberryplc.stepper.uart import TMC2208UART


def test():
    with TMC2208UART(port="/dev/ttyAMA0") as uart:
        print("Read CHOPCONF register: ")
        print(uart.read_register("CHOPCONF"))
        

if __name__ == '__main__':
    test()
