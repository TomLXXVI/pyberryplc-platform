from pyberryplc.core import SharedMemoryBlock, MemoryVariable


# Communication between main plc and other plc units
db0 = SharedMemoryBlock(
    name="DB0",
    data={
        "ProductionEnable": MemoryVariable(),  # main plc writes / other units read
    }
)


# Communication between loading station and infeed conveyor.
db1 = SharedMemoryBlock(
    name="DB1",
    data={
        "RequestConveyorAccept": MemoryVariable(),  # loading station writes / conveyor reads
        "ConveyorReadyToAccept": MemoryVariable(),  # conveyor writes / loading station reads
        "ConveyorAcceptedTray": MemoryVariable(),   # conveyor writes / loading station reads
    }
)


# Communication between plc units and SCADA
db2 = SharedMemoryBlock(
    name="DB2",
    data={
        "RegisterTrayTransfer": MemoryVariable(),
    }
)
