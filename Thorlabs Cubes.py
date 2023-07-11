import time
import clr
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.KCube.PiezoCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.KCube.StrainGaugeCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.GenericPiezoCLI import *
from Thorlabs.MotionControl.KCube.PiezoCLI import *
from Thorlabs.MotionControl.KCube.StrainGaugeCLI import *
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import *
from System import Decimal  # necessary for real world units #
import nidaqmx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import numpy as np

#Extra Variables for Translational Coding
LoopVoltage = -0.5
TargetVoltage = 74

# create new device
serial_P = "29251002"
serial_S = "59000880"
serial_A = "69252146"

def main():

    try:
        DeviceManagerCLI.BuildDeviceList()

        # Connect, begin polling, and enable
        Piezo = KCubePiezo.CreateKCubePiezo(serial_P)
        Strain = KCubeStrainGauge.CreateKCubeStrainGauge(serial_S)
        Aligner = KCubePositionAligner.CreateKCubePositionAligner(serial_A)

        Piezo.Connect(serial_P)
        Strain.Connect(serial_S)
        Aligner.Connect(serial_A)

        # Get Device Information and display description
        Piezo_info = Piezo.GetDeviceInfo()
        Strain_info = Strain.GetDeviceInfo()
        Aligner_info = Aligner.GetDeviceInfo()
                                                                #ensure successful connection of:
        print(f'Opening {Piezo_info.Description}, {serial_P}')  #Piezo
        print("----------------------------------------")
        print(f'Opening {Strain_info.Description}, {serial_S}')  #Strain Gauge
        print("----------------------------------------")
        print(f'Opening {Aligner_info.Description}, {serial_A}')  #Position Aligner
        print("----------------------------------------")

        # Start polling and enable for Piezo
        Piezo.StartPolling(250)  #250ms polling rate
        time.sleep(0.25)
        Piezo.EnableDevice()
        time.sleep(0.5)  # Wait for device to enable

        if not Piezo.IsSettingsInitialized():  # checks if piezo initializes
            Piezo.WaitForSettingsInitialized(10000)  # 10 second timeout
            assert Piezo.IsSettingsInitialized() is True
        # Start polling and enable for Position Aligner
        # 250ms polling rate
        Aligner.StartPolling(250)
        time.sleep(0.25)
        Aligner.EnableDevice()
        time.sleep(0.5)  # Wait for device to enable

        if not Aligner.IsSettingsInitialized():  # checks if Position Aligner initializes
             Aligner.WaitForSettingsInitialized(10000)
             assert Aligner.IsSettingsInitialized() is True

        # Start polling and enable for Strain Gauge
        # 250ms polling rate
        Strain.StartPolling(250)
        time.sleep(0.25)
        Strain.EnableDevice()
        time.sleep(0.5)  # Wait for device to enable

        if not Strain.IsSettingsInitialized():  # checks if Strain Gauge initializes
            Strain.WaitForSettingsInitialized(10000)
            assert Strain.IsSettingsInitialized() is True

        # Load the device configuration
        Piezo_config = Piezo.GetPiezoConfiguration(serial_P)
        Aligner_config = Aligner.GetPositionAlignerConfiguration(serial_A)
        Strain_config = Strain.GetStrainGaugeConfiguration(serial_S)

        print("Cubes Initialized")

        #initialization of data from K-cubes
        straindata = []
        Xdiff = []
        Ydiff = []
        Sum = []
        ConversionData = []
        straindatarev = []
        Xdiffrev = []
        Ydiffrev = []
        Sumrev = []
        x = []
        y = []
        i = 0
        XdiffFwd = "_XDiff-Forward-Indentation.gsac"
        YdiffFwd = "_YDiff-Forward-Indentation.gsac"
        SUMFwd = "_Sum-Forward-Indentation.gsac"
        XdiffRev = "_XDiff-Reverse-Indentation.gsac"
        YdiffRev = "_YDiff-Reverse-Indentation.gsac"
        SUMRev = "_Sum-Reverse-Indentation.gsac"

        window = tk.Tk()
        window.title("K-Cube Controller")
        window.minsize(400, 250)

        for i in range(3):
            window.columnconfigure(i, weight=1, minsize=125)
            window.rowconfigure(i, weight=1, minsize=75)
        for j in range(0, 3):
            frame = tk.Frame(master=window)
            frame.grid(row=i, column=j, padx=10, pady=10)

        def quitApp():
            # Stop Polling and Disconnect
            print("Closing the Program")
            Piezo.StopPolling()
            Piezo.Disconnect()
            Strain.StopPolling()
            Strain.Disconnect()
            Aligner.StopPolling()
            Aligner.Disconnect()
            window.destroy()

        def Kcube():
            Volt = MaxVoltage.get()
            Velocity = Speed.get()
            MaxAlignerSetpoint = Displacement.get()

            if Volt.strip() == "" and Velocity.strip() == "" and MaxAlignerSetpoint.strip() == "":
                messagebox.showinfo("Error", "Please enter values into all three parameters")

            elif Volt.strip() != "" and Velocity.strip() != "" and MaxAlignerSetpoint.strip() != "":
                Xdiff.clear()
                Ydiff.clear()
                Sum.clear()
                straindata.clear()
                Xdiffrev.clear()
                Ydiffrev.clear()
                Sumrev.clear()
                straindatarev.clear()

                figure, axis = plt.subplots(3, 2)
                fig, ax = plt.subplots()
                ax.clear()
                ax.set_xlim(1, -1)
                ax.set_ylim(-1, 1)
                ax.set_xlabel('Xdiff')
                ax.set_ylabel('Ydiff')
                ax.set_title('Position Aligner Detection Graph')
                line, = ax.plot([], [], 'o')
                channels = ["Dev1/ai0", "Dev1/ai1", "Dev1/ai2", "Dev1/ai3"]
                # Initializing and Setting Inputs for DAQ
                with nidaqmx.Task() as task:
                    for channel in channels:
                        task.ai_channels.add_ai_voltage_chan(
                            channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                    increment = 0
                    while True:
                        increment += 0.5
                        Piezo.SetOutputVoltage(Decimal(increment))
                        time.sleep(float(Velocity))
                        with nidaqmx.Task() as task:
                            for channel in channels:
                                task.ai_channels.add_ai_voltage_chan(
                                    channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                            data = task.read()
                        Xdiff.append(data[1])
                        Ydiff.append(data[0])
                        Sum.append(data[2])
                        straindata.append(2.146478922 * data[3])
                        x.append(data[1])
                        y.append(data[0])
                        line.set_data(x[-1], y[-1])
                        fig.canvas.draw()
                        plt.pause(0.1)
                        if y[-1] >= float(MaxAlignerSetpoint):
                            break
                        elif y[-1] <= -float(MaxAlignerSetpoint):
                            break
                    # plotting data into different plots
                    axis[0, 0].plot(straindata, Xdiff)
                    axis[0, 0].set_ylabel('Xdiff Voltage')
                    axis[1, 0].plot(straindata, Ydiff)
                    axis[1, 0].set_ylabel('Ydiff Voltage')
                    axis[2, 0].plot(straindata, Sum)
                    axis[2, 0].set_xlabel('Strain Gauge Displacement (um)')
                    axis[2, 0].set_ylabel('Sum Voltage')

                    while True:
                        increment -= 0.5
                        Piezo.SetOutputVoltage(Decimal(increment))
                        time.sleep(float(Velocity))
                        with nidaqmx.Task() as task:
                            for channel in channels:
                                task.ai_channels.add_ai_voltage_chan(
                                    channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                            data = task.read()
                        Xdiffrev.append(data[1])
                        Ydiffrev.append(data[0])
                        Sumrev.append(data[2])
                        straindatarev.append(2.146478922 * data[3])
                        x.append(data[1])
                        y.append(data[0])
                        line.set_data(x[-1], y[-1])
                        fig.canvas.draw()
                        plt.pause(0.1)
                        if y[-1] <= 0:
                            break

                    axis[0, 1].plot(straindatarev, Xdiffrev)
                    axis[1, 1].plot(straindatarev, Ydiffrev)
                    axis[2, 1].plot(straindatarev, Sumrev)
                    axis[2, 1].set_xlabel('Strain Gauge Displacement (um)')
                    plt.show()
            else:
                messagebox.showinfo("Error", "Please enter a values into the missing parameters ")

        def Setzero():
            Piezo.SetZero()
            Strain.SetZero()
            time.sleep(10)

        def SaveTxt():

            with open(XdiffFwd, "w") as file:
                file.write("# Strain Data" + "\t" + "XDiff Data" + "\n")
                i = 0
                while i < len(straindata):
                    file.write(str(straindata[i]) + " " + str(Xdiff[i]) + "\n")
                    i += 1
            file.close()

            with open(YdiffFwd, "w") as file:
                file.write("# Strain Data" + "\t" + "YDiff Data" + "\n")
                i = 0
                while i < len(straindata):
                    file.write(str(straindata[i]) + " " + str(Ydiff[i]) + "\n")
                    i += 1
            file.close()

            with open(SUMFwd, "w") as file:
                file.write("# Strain Data" + "\t" + "Sum Data" + "\n")
                i = 0
                while i < len(straindata):
                    file.write(str(straindata[i]) + " " + str(Sum[i]) + "\n")
                    i += 1
            file.close()

            with open(XdiffRev, "w") as file:
                file.write("# Strain Data" + "\t" + "XDiff Data" + "\n")
                i = len(straindatarev) - 1
                while i >=0:
                    file.write(str(straindatarev[i]) + " " + str(Xdiffrev[i]) + "\n")
                    i -= 1
            file.close()

            with open(YdiffRev, "w") as file:
                file.write("# Strain Data" + "\t" + "YDiff Data" + "\n")
                i = len(straindatarev) - 1
                while i >=0:
                    file.write(str(straindatarev[i]) + " " + str(Ydiffrev[i]) + "\n")
                    i -= 1
            file.close()

            with open(SUMRev, "w") as file:
                file.write("# Strain Data" + "\t" + "Sum Data" + "\n")
                i = len(straindatarev) - 1
                while i >= 0:
                    file.write(str(straindatarev[i]) + " " + str(Sumrev[i]) + "\n")
                    i -= 1
            file.close()

        def Conversion():
            i = 0
            ConversionTest = "CorrectedVoltage.gsac"
            ConversionFactor = "CorrectionFactor.txt"

            with open(ConversionFactor, "r") as file:
                incrementalconversion = [float(line.strip()) for line in file]
            file.close()

            while i < len(incrementalconversion) and i < len(straindata):
                ConversionData.append(straindata[int(i)] * incrementalconversion[int(i)])
                i += 1

            with open(ConversionTest, "w") as file:
                file.write("# Strain Data" + "\t" + "XDiff Data" + "\n")
                i = 0
                while i < len(ConversionData):
                    file.write(str(ConversionData[i]) + "\n")
                    i += 1
            file.close()
            ConversionData.clear()

        def PositionMap():
            setpoint = 0.6
            Volt = MaxVoltage.get()
            Velocity = Speed.get()
            MaxAlignerSetpoint = Displacement.get()

            if Volt.strip() == "" and Velocity.strip() == "" and MaxAlignerSetpoint.strip() == "":
                messagebox.showinfo("Error", "Please enter values into all three parameters")

            elif Volt.strip() != "" and Velocity.strip() != "" and MaxAlignerSetpoint.strip() != "":
                Aligner.SetOperatingMode(PositionAlignerStatus.OperatingModes.Monitor, True)
                channels = ["Dev1/ai0", "Dev1/ai1", "Dev1/ai2", "Dev1/ai3"]
                # Initializing and Setting Inputs for DAQ
                with nidaqmx.Task() as task:
                    for channel in channels:
                        task.ai_channels.add_ai_voltage_chan(
                            channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                    data = task.read()
                x.append(data[1])
                y.append(data[0])
                Sum.append(data[2])
                PositionSquare = "CentralSquare.txt"
                with open(PositionSquare, "r") as file:
                    Square = [[float(item) for item in line.strip().split('\t')] for line in file]
                file.close()
                q = 0
                xx = []
                yy = []
                while q < len(Square):
                    Tx, Ty = Square[q]
                    xx.append(Tx)
                    yy.append(Ty)
                    q += 1

                root = tk.Tk()
                root.title("Position Aligner")

                canvas = tk.Canvas(root, width=400, height=300)
                canvas.grid(row=0, column=0, padx=10, pady=10)

                progress = ttk.Progressbar(root, length=200, mode='determinate', orient="vertical")
                progress.grid(row=0, column=1, padx=10, pady=10)

                fig, ax = plt.subplots()
                ax.clear()
                ax.set_xlim(5, -5)
                ax.set_ylim(-5, 5)
                ax.set_xlabel('Xdiff')
                ax.set_ylabel('Ydiff')
                ax.set_title('Position Aligner Detection Graph')
                line, = ax.plot(x[-1], y[-1], 'o')
                point, = ax.plot(xx, yy, ':')

                canvas = FigureCanvasTkAgg(fig, master=canvas)
                canvas.draw()
                canvas.get_tk_widget().grid(row=0, column=0, padx=10, pady=10)

                increment = 0

                while True:
                    increment += 0.5
                    Piezo.SetOutputVoltage(Decimal(increment))
                    time.sleep(float(Velocity))
                    with nidaqmx.Task() as task:
                        for channel in channels:
                            task.ai_channels.add_ai_voltage_chan(
                                channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                        data = task.read()
                    x.append(data[1])
                    y.append(data[0])
                    Sum.append(data[2])
                    line.set_data(x[-1], y[-1])
                    ax.relim()
                    ax.autoscale_view()
                    canvas.draw()
                    if y[-1] >= float(MaxAlignerSetpoint):
                        break
                    progress['value'] = (Sum[-1] * 10)
                    window.update()

                while True:
                    increment -= 0.5
                    Piezo.SetOutputVoltage(Decimal(increment))
                    time.sleep(float(Velocity))
                    with nidaqmx.Task() as task:
                        for channel in channels:
                            task.ai_channels.add_ai_voltage_chan(
                                channel)  # adds XDiff, YDiff, Sum, and Strain Gauge Monitor to the NI DAQ Initialization

                        data = task.read()
                    x.append(data[1])
                    y.append(data[0])
                    Sum.append(data[2])
                    line.set_data(x[-1], y[-1])
                    ax.relim()
                    ax.autoscale_view()
                    fig.canvas.draw()
                    plt.pause(0.01)
                    if y[-1] <= 0:
                        root.destroy()
                        break
                    progress['value'] = (Sum[-1] * 10)
                    window.update()

                PositionMap()

            else:
                messagebox.showinfo("Error", "Please enter a values into the missing parameters ")
            #plt.show()
            #time.sleep(500)

        MaxVoltage = tk.Entry(window, bd=6, width=15)
        MaxVoltage.grid(column=0, row=1, columnspan=2, padx=10, pady=10)
        tk.Label(window, text=": Set Maximum Voltage").grid(column=1, row=1, columnspan=2, padx=10, pady=10)

        Speed = tk.Entry(window, bd=6, width=15)
        Speed.grid(column=0, row=0, columnspan=2, padx=10, pady=10)
        tk.Label(window, text=": Set Maximum Speed" + '\n' + "(0.0001-0.1)").grid(column=1, row=0, columnspan=2, padx=10, pady=10)

        Displacement = tk.Entry(window, bd=6, width=15)
        Displacement.grid(column=0, row=2, columnspan=2, padx=10, pady=10)
        tk.Label(window, text=": Set Position Aligner" + '\n'+ "Setpoint (0-1)").grid(column=1, row=2, columnspan=2, padx=10, pady=10)

        QuitButton = tk.Button(window, bd=5, bg="#ff0000", text="Quit", height=3, width=15, command=quitApp)
        QuitButton.grid(column=3, row=3, padx=10, pady=10)

        SubmitButton = tk.Button(window, bd=5, bg="#90EE90", text="Start", height=3, width=15, command=Kcube)
        SubmitButton.grid(column=3, row=0, padx=10, pady=10)

        Zero = tk.Button(window, bd=5, bg="#E0B0FF", text="Zero", height=3, width=15, command=Setzero)
        Zero.grid(column=3, row=1, padx=10, pady=10)

        Save = tk.Button(window, bd=5, bg="#E0B0FF", text="Save to Txt File", height=3, width=15, command=SaveTxt)
        Save.grid(column=3, row=2, padx=10, pady=10)

        Convert = tk.Button(window, bd=5, bg="#abdbd9", text="V > nN", height=1, width=10, command=Conversion)
        Convert.grid(column=0, row=3, columnspan=2, padx=10, pady=10)

        Align = tk.Button(window, bd=5, bg="#abdbd9", text="Position Map", height=1, width=10, command=PositionMap)
        Align.grid(column=1, row=3, columnspan=2, padx=10, pady=10)

        #PositionMap()
        window.mainloop()

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()