import sys
import time

from vectornav import Registers, Sensor, ByteBuffer
from vectornav.Plugins import Logger


def main(argv):

    #TODO: Figure out logging

    # Define the port connection parameters
    portName = argv[0] if argv else "COM5"  # Change the sensor port name to the com port of your local machine
    filePath = "log.bin"

    # 1. Instantiate a Sensor object and use it to connect to the VectorNav unit
    sensor = Sensor()
    try:
        sensor.autoConnect(portName)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when connecting to {portName}.\n")
        return
    print(f"Connected to {portName} at {sensor.connectedBaudRate()}")

    # 2. Poll and print the model number using a read register command
    # Create an empty register object of the necessary type, where the data member will be populated when the sensor responds to the "read register" request
    modelRegister = Registers.System.Model()
    try:
        sensor.readRegister(modelRegister)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when reading register 1 (Model).\n")
        return
    print(f"Sensor Model Number: {modelRegister.model}")

    bufferToLog = ByteBuffer(1024*3)
    logger = Logger.SimpleLogger(bufferToLog, filePath)

    sensor.registerReceivedByteBuffer(bufferToLog)

    if logger.start():
        print("Error: Failed to write to file")

    print(f"Logging to {filePath}")

    # 3. Poll and print the current yaw, pitch, and roll using a read register command
    yprRegister = Registers.Attitude.YawPitchRoll()
    try:
        sensor.readRegister(yprRegister)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when reading 8 (YawPitchRoll).\n")
        return
    print(f"Current Reading:\n\tYaw: {yprRegister.yaw} , Pitch: {yprRegister.pitch} , Roll: {yprRegister.roll} ")

    # 4. Configure the asynchronous ASCII output to YPR at 2Hz
    asyncDataOutputType = Registers.System.AsyncOutputType()
    asyncDataOutputType.ador = Registers.System.AsyncOutputType.Ador.YPR
    asyncDataOutputType.serialPort = Registers.System.AsyncOutputType.SerialPort.Serial1
    try:
        sensor.writeRegister(asyncDataOutputType)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when writing register 6 (AsyncOutputType).\n")
        return
    print("ADOR Configured")

    asyncDataOutputFrequency = Registers.System.AsyncOutputFreq()
    asyncDataOutputFrequency.adof = Registers.System.AsyncOutputFreq.Adof.Rate2Hz
    asyncDataOutputFrequency.serialPort = Registers.System.AsyncOutputFreq.SerialPort.Serial1
    try:
        sensor.writeRegister(asyncDataOutputFrequency)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when writing register 7 (AsyncOutputFreq).\n")
        return
    print("ADOF Configured")

    # 5. Configure the first binary output message to output timeStartup, accel, and angRate, all from common group, at a 2 Hz output rate (1 Hz if VN-300) through both serial ports
    binaryOutput1Register = Registers.System.BinaryOutput1()
    binaryOutput1Register.asyncMode.serial1 = 1
    binaryOutput1Register.asyncMode.serial2 = 1
    binaryOutput1Register.rateDivisor = 400
    binaryOutput1Register.common.timeStartup = 1
    binaryOutput1Register.common.accel = 1
    binaryOutput1Register.common.deltas = 1
    binaryOutput1Register.common.angularRate = 1
    binaryOutput1Register.common.imu = 1

    try:
        sensor.writeRegister(binaryOutput1Register)
    except Exception as latestError:
        print(f"Error: {latestError} encountered when writing register 75 (BinaryOutput1).\n")
        return
    print("Binary output 1 message configured")

    # 6. Enter a loop for 5 seconds where it:
    #      Determines which measurement it received (VNYPR or the necessary binary header)
    #      Prints out the relevant measurement from the CompositeData struct

    ###########################
    # Jeevan - 12/08
    ##########################
    # Data to be collected - 
    # deltaTheta
    # deltaVel
    # accel
    # angularRate

    t0 = time.time()
    dt0 = 0
    dtf = 0
    counter = 0
    while time.time() - t0 < 5:
        compositeData = sensor.getNextMeasurement()
        # Check to make sure that a measurement is available
        if not compositeData:
            continue

        if compositeData.matchesMessage(binaryOutput1Register):
            print("Found binary 1 measurement.")

            timeStartup = compositeData.time.timeStartup.nanoseconds()
            print(f"\tTime: {timeStartup}")

            accel = compositeData.imu.accel
            print(f"\tAccel X: {accel[0]}\n\tAccel Y: {accel[1]}\n\tAccel Z: {accel[2]}")

            angular = compositeData.imu.angularRate
            print(f"\tAngular X: {angular[0]}\n\tAngular Y: {angular[1]}\n\tAngular Z: {angular[1]}")
            
            deltaTheta = compositeData.imu.deltaTheta
            print(f"\tDelta Time: {deltaTheta.deltaTime}")

            deltaTheta = compositeData.imu.deltaTheta
            print(f"\tDelta Theta X: {deltaTheta.deltaTheta[0]}\n\tDelta Theta Y: {deltaTheta.deltaTheta[1]}\n\tDelta Theta Z: {deltaTheta.deltaTheta[2]}")

            if counter == 0:
                dt0 = deltaTheta.deltaTheta[0]

            dtf = deltaTheta.deltaTheta[0]


            deltaVel = compositeData.imu.deltaVel
            print(f"\tDelta Vel X: {deltaVel[0]}\n\tDelta Vel Y: {deltaVel[1]}\n\tDelta Vel Z: {deltaVel[2]}")
        elif compositeData.matchesMessage("VNYPR"):
            print("Found ASCII YPR measurement.")

            ypr = compositeData.attitude.ypr
            print(f"\tYaw: {ypr.yaw}\n\tPitch: {ypr.pitch}\n\tRoll: {ypr.roll}")
        else:
            print("Unrecognized asynchronous message received")

        # Handle asynchronous errors
        try:
            sensor.throwIfAsyncError()
        except Exception as asyncError:
            print(f"Received async error: {asyncError}");

        counter += 1
  
    logger.stop()
    sensor.deregisterReceivedByteBuffer()

    print(f"Logged {logger.numBytesLogged()} bytes")

    # 7. Disconnect from the VectorNav unit
    sensor.disconnect()
    print("Sensor disconnected")
    print("GettingStarted example complete.")

    

    print(f"\n\tInit: {dt0}\n\tFinal: {dtf}")


if __name__ == "__main__":
    main(sys.argv[1:])
