from pyberryplc.core import SharedMemoryBlock, MemoryVariable


# Communication between main plc and other plc units
db0 = SharedMemoryBlock(
    name="DB0",
    data={
        "ProductionEnable": MemoryVariable(),    # main plc writes / other units read
        "ExitFlag": MemoryVariable(),                # main plc writes / other units read
        "EmergencyStopActive": MemoryVariable(),
    }
)


# Communication between loading station and infeed conveyor.
db1 = SharedMemoryBlock(
    name="DB1",
    data={
        "RequestConveyorAccept": MemoryVariable(),   # loading station writes / conveyor reads
        "ConveyorReadyToAccept": MemoryVariable(),   # conveyor writes / loading station reads
        "TrayTransferDone": MemoryVariable(),        # loading station writes / conveyor reads
        "ConveyorFaultActive": MemoryVariable(),     # conveyor writes / loading station reads
    }
)
