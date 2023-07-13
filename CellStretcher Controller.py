import time
import clr
import numpy as np

# ---------------------------------------------------------------------------------------
# Camera Imports ------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------
import cv2
framerate = 30

#img = cv2.imread(r"C:\Users\CTELab\Pictures\Screenshots\Screenshot 2023-06-21 091959.png")

# ---------------------------------------------------------------------------------------
# Thorlab K-Cube Imports ----------------------------------------------------------------
# ---------------------------------------------------------------------------------------
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.KCube.DCServoCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.DCServoCLI import *
from Thorlabs.MotionControl.GenericMotorCLI.ControlParameters import *

# ---------------------------------------------------------------------------------------
# System Variables and UI Creation Imports ----------------------------------------------
# ---------------------------------------------------------------------------------------
from System import Decimal  # necessary for real world units #
import tkinter as tk
from tkinter.ttk import Frame
from PIL import ImageTk, Image
from tkinter import messagebox
from zaber.serial import AsciiSerial, AsciiDevice, AsciiCommand, AsciiReply, UnexpectedReplyError

# ---------------------------------------------------------------------------------------
# Global Variables ----------------------------------------------------------------------
# ---------------------------------------------------------------------------------------
serial_Barrel = "27262176"
serial_Mini = "27601948"
serial_Zaber = "COM3"
serial_Z = ""
Motor1Speed = 0
Motor1Accel = 0
Motor1Displ = 0
Motor2Speed = 0
Motor2Accel = 0
Motor2Displ = 0
Motor3Speed = 0
Motor3Accel = 0
Motor3Displ = 0
Motor1Jog = 0.1
Motor2Jog = 0.5
InitialPDMSDistance = 14

port = AsciiSerial(serial_Zaber)
Zaber = AsciiDevice(port, 1)

# ---------------------------------------------------------------------------------------
# Main Loop -----------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------


def main():
    try:
        DeviceManagerCLI.BuildDeviceList()

        # Connect, begin polling, and enable
        Barrel = KCubeDCServo.CreateKCubeDCServo(serial_Barrel)
        Mini = KCubeDCServo.CreateKCubeDCServo(serial_Mini)

        Barrel.Connect(serial_Barrel)
        Mini.Connect(serial_Mini)

        # Get Device Information and display description
        Barrel_info = Barrel.GetDeviceInfo()
        Mini_info = Mini.GetDeviceInfo()
        # ensure successful connection of:
        print(f'Opening {Barrel_info.Description}, {serial_Barrel}')  # Piezo
        print("----------------------------------------")
        print(f'Opening {Mini_info.Description}, {serial_Mini}')  # Strain Gauge
        print("----------------------------------------")
        print(f'Opening device Zaber Stage, {serial_Zaber}')
        print("----------------------------------------")

        # Start polling and enable for Piezo
        Barrel.StartPolling(250)  # 250ms polling rate
        time.sleep(0.25)
        Barrel.EnableDevice()
        time.sleep(0.5)  # Wait for device to enable

        if not Barrel.IsSettingsInitialized():  # checks if piezo initializes
            Barrel.WaitForSettingsInitialized(10000)  # 10 second timeout
            assert Barrel.IsSettingsInitialized() is True

        # Start polling and enable for Position Aligner
        # 250ms polling rate
        Mini.StartPolling(250)
        time.sleep(0.25)
        Mini.EnableDevice()
        time.sleep(0.5)  # Wait for device to enable

        if not Mini.IsSettingsInitialized():  # checks if Position Aligner initializes
            Mini.WaitForSettingsInitialized(10000)
            assert Mini.IsSettingsInitialized() is True

        print("Cubes Initialized")

        # ---------------------------------------------------------------------------------------
        # Obtain Previous Device Settings and Update to Concurrent ------------------------------
        # ---------------------------------------------------------------------------------------

        Barrel_Config = Barrel.LoadMotorConfiguration(serial_Barrel,
                                                      DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings)
        Mini_Config = Mini.LoadMotorConfiguration(serial_Mini,
                                                  DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings)
        Barrel_Config.DeviceSettingsName = "Base Stage"
        Mini_Config.DeviceSettingsName = "Flexi Focusing"
        Barrel_Config.UpdateCurrentConfiguration()
        Mini_Config.UpdateCurrentConfiguration()
        Barrel.SetSettings(Barrel.MotorDeviceSettings, True, False)
        Mini.SetSettings(Mini.MotorDeviceSettings, True, False)

        # ---------------------------------------------------------------------------------------
        # Tkinter Window Creation ---------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        window = tk.Tk()
        window.title("K-Cube Controller")
        window.minsize(400, 250)
        x = 4
        for i in range(5):
            window.columnconfigure(i, weight=1, minsize=125)
            window.rowconfigure(x, weight=1, minsize=75)
        for j in range(0, 5):
            frames = tk.Frame(master=window)
            frames.grid(row=x, column=j, padx=10, pady=10)
        # ---------------------------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        def EnlargedVideo(event=None):
            window.withdraw()
            Popout = tk.Toplevel(window)
            Popout.geometry("640x512")
            canvas = tk.Canvas(Popout)
            canvas.pack(fill=tk.BOTH, expand=True)

            def updating():
                ret, frame = camera.read()
                if not ret:
                    messagebox.showerror(title='Imaging Failure',message="The Camera Couldn't Load, Click the Image to Retry")
                    return

                global image
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                image = Image.fromarray(gray)
                resize_image()
                if Popout.winfo_exists():
                    Popout.update()
                    Popout.after(1, updating)

            def resize_image():
                if canvas.winfo_exists():
                    width = canvas.winfo_width()
                    height = canvas.winfo_height()
                    resize_img = image.resize((width, height))
                    fixed = ImageTk.PhotoImage(resize_img)
                    canvas.create_image(0, 0, anchor=tk.NW, image=fixed)
                    canvas.image = fixed

            def canvas_resize(event):
                resize_image()

            def popoutclose():
                window.deiconify()
                Popout.destroy()

            canvas.bind("<Configure>", canvas_resize)
            Popout.protocol("WM_DELETE_WINDOW", popoutclose)

            Popout.after(1, updating)
            Popout.mainloop()

        def quitApp():
            # Stop polling and disconnect devices
            print("Closing the Program")
            Barrel.StopPolling()
            Barrel.Disconnect()
            Mini.StopPolling()
            Mini.Disconnect()
            window.destroy()

        def Cyclic():
            Cycles = Cycle.get()
            StrainRate = Strain.get()
            if 0 < int(StrainRate) <= 107 and int(Cycles) > 0:
                Velocity = int(StrainRate) * InitialPDMSDistance
                Frequency = Velocity / (np.pi * 12)
                Period = (1/Frequency)*1000
                try:
                    #Properties to send: Movement_ _sin(cyclical motion)_ _Amplitude(nm)_ _time(ms; keep above 27.2ms/cycle)_ _Cycles(Qty/ #)
                    Zaber.send("move sin 6000000 {} {}".format(Period, Cycles))
                except UnexpectedReplyError as e:
                    print("Reply: {}".format(e))
                print("Zaber Moved Successfully")
            else:
                messagebox.showerror(message="Please Insert Proper Values into Cycle" + '\n' + "and Strain Rate Text Boxes:" + '\n' + " 0 < Cycles" + '\n' + "0 < Strain Rate (/s) < 107")

        def Setzero():
            print("Homing Devices")
            Barrel.Home(60000)
            Mini.Home(60000)
            print("Devices Homed")
            try:
                Zaber.send("home")
                print("Zaber Homed Successfully")
            except UnexpectedReplyError as e:
                print("Received reply: {}".format(e.reply))

        def Position():
            global Motor1Displ, Motor1Speed, Motor1Accel, Motor2Displ, Motor2Speed, Motor2Accel, Motor3Displ, Motor3Speed, Motor3Accel
            Motor1Displ = Position1.get()
            Motor1Speed = Speed1.get()
            Motor1Accel = Acceleration1.get()
            Motor2Displ = Position2.get()
            Motor2Speed = Speed2.get()
            Motor2Accel = Acceleration2.get()
            Motor3Displ = Position3.get()
            Motor3Speed = Speed3.get()
            Motor3Accel = Acceleration3.get()

            if Motor1Displ.strip() == "" and Motor1Speed.strip() == "" and Motor1Accel.strip() == "" and Motor2Displ.strip() == "" and Motor2Speed.strip() == "" and Motor2Accel.strip() == "" and Motor3Displ.strip() == "" and Motor3Speed.strip() == "" and Motor3Accel.strip() == "":
                messagebox.showinfo(message="Please input values into the motor parameters")

            elif Motor1Displ.strip() != "" and Motor1Speed.strip() != "" and Motor1Accel.strip() != "" and Motor2Displ.strip() != "" and Motor2Speed.strip() != "" and Motor2Accel.strip() != "" and Motor3Displ.strip() != "" and Motor3Speed.strip() != "" and Motor3Accel.strip() != "":
                print("Stretching and Moving Flexiscope")
                Barrel.SetJogStepSize(Decimal(Motor1Jog))
                Barrel.SetJogVelocityParams(Decimal(float(Motor1Speed)), Decimal(float(Motor1Accel)))
                Barrel.MoveTo(Decimal(float(Motor1Displ)), 60000)
                Mini.SetJogStepSize(Decimal(Motor2Jog))
                Mini.SetJogVelocityParams(Decimal(float(Motor2Speed)), Decimal(float(Motor2Accel)))
                Mini.MoveTo(Decimal(float(Motor2Displ)), 60000)
                Barrel.MoveJog(MotorDirection.Forward, 60000)
                Mini.MoveJog(MotorDirection.Forward, 60000)
                Zaber.move_vel(int(Motor3Speed), blocking=False)
                Zaber.move_abs(int(Motor3Displ), blocking=True)

            elif Motor1Displ.strip() != "" and Motor1Speed.strip() != "" and Motor1Accel.strip() != "" and Motor2Displ.strip() != "" and Motor2Speed.strip() != "" and Motor2Accel.strip() != "":
                tk.Label(window,text="" + '\n' + str(Motor1Displ) + " mm" + '\n' + str(Motor1Speed) + " mm/s" + '\n' + str(Motor1Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=0, padx=5, pady=5,sticky="E")
                tk.Label(window,text="" + '\n' + str(Motor2Displ) + " mm" + '\n' + str(Motor2Speed) + " mm/s" + '\n' + str(Motor2Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5,sticky="E")
                Barrel.SetJogStepSize(Decimal(Motor1Jog))
                Barrel.SetJogVelocityParams(Decimal(float(Motor1Speed)), Decimal(float(Motor1Accel)))
                Barrel.MoveTo(Decimal(float(Motor1Displ)), 60000)
                Mini.SetJogStepSize(Decimal(Motor2Jog))
                Mini.SetJogVelocityParams(Decimal(float(Motor2Speed)), Decimal(float(Motor2Accel)))
                Mini.MoveTo(Decimal(float(Motor2Displ)), 60000)
                Barrel.MoveJog(MotorDirection.Forward, 60000)
                Mini.MoveJog(MotorDirection.Forward, 60000)

            elif Motor1Displ.strip() != "" and Motor1Speed.strip() != "" and Motor1Accel.strip() != "" and Motor3Displ.strip() != "" and Motor3Speed.strip() != "" and Motor3Accel.strip() != "":
                tk.Label(window,text="" + '\n' + str(Motor1Displ) + " mm" + '\n' + str(Motor1Speed) + " mm/s" + '\n' + str(Motor1Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=0, padx=5, pady=5,sticky="E")
                Barrel.SetJogStepSize(Decimal(Motor1Jog))
                Barrel.SetJogVelocityParams(Decimal(float(Motor1Speed)), Decimal(float(Motor1Accel)))
                Barrel.MoveTo(Decimal(float(Motor1Displ)), 60000)
                Barrel.MoveJog(MotorDirection.Forward, 60000)
                tk.Label(window, text="" + '\n' + str(Motor3Displ) + " mm" + '\n' + str(Motor3Speed) + " mm/s" + '\n' + str(Motor3Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5,sticky="E")
                try:
                    Zaber.move_vel(int(Motor3Speed), blocking=False)
                    Zaber.move_abs(int(Motor3Displ) * 1000000, blocking=True)
                    print("Zaber Moved Successfully")
                except UnexpectedReplyError as e:
                    print("Received reply: {}".format(e.reply))

            elif Motor1Displ.strip() != "" and Motor1Speed.strip() != "" and Motor1Accel.strip() != "" and Motor2Displ.strip() != "" and Motor2Speed.strip() != "" and Motor2Accel.strip() != "":
                tk.Label(window,text="" + '\n' + str(Motor2Displ) + " mm" + '\n' + str(Motor2Speed) + " mm/s" + '\n' + str(Motor2Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5,sticky="E")
                Mini.SetJogStepSize(Decimal(Motor2Jog))
                Mini.SetJogVelocityParams(Decimal(float(Motor2Speed)), Decimal(float(Motor2Accel)))
                Mini.MoveTo(Decimal(float(Motor2Displ)), 60000)
                Mini.MoveJog(MotorDirection.Forward, 60000)
                tk.Label(window, text="" + '\n' + str(Motor3Displ) + " mm" + '\n' + str(Motor3Speed) + " mm/s" + '\n' + str(Motor3Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5,sticky="E")
                try:
                    Zaber.move_vel(int(Motor3Speed), blocking=False)
                    Zaber.move_abs(int(Motor3Displ) * 1000000, blocking=True)
                    print("Zaber Moved Successfully")
                except UnexpectedReplyError as e:
                    print("Received reply: {}".format(e.reply))

            elif Motor1Displ.strip() != "" and Motor1Speed.strip() != "" and Motor1Accel.strip() != "":
                if float(Motor1Displ) > 25 or float(Motor1Speed) > 2.2 or float(Motor1Accel) > 1.5 or float(Motor1Displ) < 0 or float(Motor1Speed) < 0 or float(Motor1Accel) < 0:
                    messagebox.showerror(message="The Parameters are Invalid, try to keep:" + '\n' + "0 < Displacement < 25 mm" + '\n' + "0 < Speed < 2.2 mm/s" + '\n' + "0 < Acceleration < 1.5 mm/s^2")

                else:
                    print("Adjusting Flexiscope Viewing Area")
                    tk.Label(window, text="" + '\n' + str(Motor1Displ) + " mm" + '\n' + str(Motor1Speed) + " mm/s" + '\n' + str(Motor1Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=0, padx=5, pady=5, sticky="E")
                    Barrel.SetJogStepSize(Decimal(Motor1Jog))
                    Barrel.SetJogVelocityParams(Decimal(float(Motor1Speed)), Decimal(float(Motor1Accel)))
                    Barrel.MoveTo(Decimal(float(Motor1Displ)), 60000)
                    Barrel.MoveJog(MotorDirection.Forward, 60000)

            elif Motor2Displ.strip() != "" and Motor2Speed.strip() != "" and Motor2Accel.strip() != "":
                if float(Motor2Displ) > 25 or float(Motor2Speed) > 2.4 or float(Motor2Accel) > 4.5 or float(Motor2Displ) < 0 or float(Motor2Speed) < 0 or float(Motor2Accel) < 0:
                    messagebox.showerror(message="The Parameters are Invalid, try to keep:" + '\n' + "0 < Displacement < 25 mm" + '\n' + "0 < Speed < 2.4 mm/s" + '\n' + "0 < Acceleration < 4.5 mm/s^2")

                else:
                    print("Adjusting Flexiscope Focus")
                    tk.Label(window, text="" + '\n' + str(Motor2Displ) + " mm" + '\n' + str(Motor2Speed) + " mm/s" + '\n' + str(Motor2Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5, sticky="E")
                    Mini.SetJogStepSize(Decimal(Motor2Jog))
                    Mini.SetJogVelocityParams(Decimal(float(Motor2Speed)), Decimal(float(Motor2Accel)))
                    Mini.MoveTo(Decimal(float(Motor2Displ)), 60000)
                    Mini.MoveJog(MotorDirection.Forward, 60000)

            elif Motor3Displ.strip() != "" and Motor3Speed.strip() != "" and Motor3Accel.strip() != "":
                if float(Motor3Displ) > 12 or float(Motor3Speed) > 1400 or float(Motor3Accel) > 245000 or float(Motor3Displ) < -12 or float(Motor3Speed) < 0 or float(Motor3Accel) < 0:
                    messagebox.showerror(message="The Parameters are Invalid, try to keep:" + '\n' + "0 < Displacement < 12 mm" + '\n' + "0 < Speed < 1400 mm/s" + '\n' + "0 < Acceleration < 245 m/s^2")

                else:
                    print("Stretching PDMS")
                    tk.Label(window, text="" + '\n' + str(Motor3Displ) + " mm" + '\n' + str(Motor3Speed) + " mm/s" + '\n' + str(Motor3Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5, sticky="E")
                    ConvertSpeed = int(Motor3Speed) / 0.000000610351527757841
                    ConvertDispl = int(Motor3Displ) * 1000000
                    ZaberSend = "move abs " + str(ConvertDispl) + " " + str(int(ConvertSpeed))
                    print(ZaberSend)
                    try:
                        Zaber.send(ZaberSend)
                        #Zaber.send("move abs {} {}".format(str(ConvertDispl), str(ConvertSpeed)))
                        #print(int(int(Motor3Speed) / 0.000000610351527757841))
                        #print("reply: " + str(reply))
                        #Zaber.move_abs(int(Motor3Displ) * 1000000, blocking=False)
                        #int(Motor3Speed) / 0.000000610351527757841
                        print("Zaber Moved Successfully")
                    except UnexpectedReplyError as e:
                        print("Received reply: {}".format(e.reply))

            else:
                messagebox.showerror("Try Entering More Values Into The Stage Parameters")

        # ---------------------------------------------------------------------------------------
        # Motor 1 Controller --------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        Position1 = tk.Entry(window, bd=6, width=8)
        Position1.grid(column=0, row=0, padx=10, pady=10, sticky="S")

        Speed1 = tk.Entry(window, bd=6, width=8)
        Speed1.grid(column=1, row=0, padx=10, pady=10, sticky="S")

        Acceleration1 = tk.Entry(window, bd=6, width=8)
        Acceleration1.grid(column=2, row=0, padx=10, pady=10, sticky="S")

        tk.Label(window, text="" + '\n' + "Set Position" + '\n' + "(0 - 25 mm):", justify="left").grid(column=0, row=0,padx=10, pady=10,sticky="N")
        tk.Label(window, text="" + '\n' + "Set Speed" + '\n' + "(0 - 2.2 mm/s):", justify="left").grid(column=1, row=0,padx=10, pady=10,sticky="N")
        tk.Label(window, text="" + '\n' + "Set Acceleration" + '\n' + "(0 - 1.5 mm/s):", justify="left").grid(column=2,row=0,padx=10,pady=10,sticky="N")
        tk.Label(window, text="" + '\n' + "Position :" + '\n' + "Speed :" + '\n' + "Accel:", justify="left").grid(column=3, row=0, padx=5, pady=5, sticky="SW")
        tk.Label(window, text="" + '\n' + str(Motor1Displ) + " mm" + '\n' + str(Motor1Speed) + " mm/s" + '\n' + str(Motor1Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=0, padx=5, pady=5, sticky="SE")

        # ---------------------------------------------------------------------------------------
        # Motor 2 Controller --------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        Position2 = tk.Entry(window, bd=6, width=8)
        Position2.grid(column=0, row=1, padx=10, pady=10, sticky="S")

        Speed2 = tk.Entry(window, bd=6, width=8)
        Speed2.grid(column=1, row=1, padx=10, pady=10, sticky="S")

        Acceleration2 = tk.Entry(window, bd=6, width=8)
        Acceleration2.grid(column=2, row=1, padx=10, pady=10, sticky="S")

        tk.Label(window, text="Set Position" + '\n' + "(0 - 25 mm):", justify="left").grid(column=0, row=1, padx=10,pady=10, sticky="N")
        tk.Label(window, text="Set Movement Speed" + '\n' + "(0 - 2.4 mm/s):", justify="left").grid(column=1, row=1,padx=10, pady=10,sticky="N")
        tk.Label(window, text="Set  Acceleration" + '\n' + "(0 - 4.5 mm/s):", justify="left").grid(column=2, row=1,padx=10, pady=10,sticky="N")
        tk.Label(window, text="" + '\n' + "Position :" + '\n' + "Speed :" + '\n' + "Accel:", justify="left").grid(column=3, row=1, padx=5, pady=5, sticky="SW")
        tk.Label(window, text="" + '\n' + str(Motor2Displ) + " mm" + '\n' + str(Motor2Speed) + " mm/s" + '\n' + str(Motor2Accel) + " mm/s^2" + "", justify="right").grid(column=3, row=1, padx=5, pady=5, sticky="SE")

        # ---------------------------------------------------------------------------------------
        # Motor 3 Controller --------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        Position3 = tk.Entry(window, bd=6, width=8)
        Position3.grid(column=0, row=2, padx=10, pady=10, sticky="S")

        Speed3 = tk.Entry(window, bd=6, width=8)
        Speed3.grid(column=1, row=2, padx=10, pady=10, sticky="S")

        Acceleration3 = tk.Entry(window, bd=6, width=8)
        Acceleration3.grid(column=2, row=2, padx=10, pady=10, sticky="S")

        tk.Label(window, text="Set Position" + '\n' + "(0 - 12 mm):", justify="left").grid(column=0, row=2, padx=10,pady=10, sticky="N")
        tk.Label(window, text="Set Movement Speed" + '\n' + "(0 - 1400 mm/s):", justify="left").grid(column=1, row=2,padx=10, pady=10,sticky="N")
        tk.Label(window, text="Set  Acceleration" + '\n' + "(0 - 4.5 mm/s):", justify="left").grid(column=2, row=2,padx=10, pady=10,sticky="N")
        tk.Label(window, text="" + '\n' + '\n' + "Position :" + '\n' + "Speed :" + '\n' + "Accel:",justify="left").grid(column=3, row=2, padx=5, pady=5, sticky="W")
        tk.Label(window,text="" + '\n' + '\n' + str(Motor3Displ) + " mm" + '\n' + str(Motor3Speed) + " mm/s" + '\n' + str(Motor3Accel) + " mm/s^2"  "", justify="right").grid(column=3, row=2, padx=5, pady=5, sticky="E")

        # ---------------------------------------------------------------------------------------
        # Cyclic Controller----------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        Cycle = tk.Entry(window, bd=6, width=8)
        Cycle.grid(column=4, row=2, padx=10, pady=10, sticky="S")

        Strain = tk.Entry(window, bd=6, width=8)
        Strain.grid(column=5, row=2, padx=10, pady=10, sticky="S")

        tk.Label(window, text="Number of" + '\n' + "Cycles:").grid(column=4, row=2, padx=10, pady=10, sticky="N")
        tk.Label(window, text="Strain Rate" + '\n' + "( 0 - 100 /s):").grid(column=5, row=2, padx=10, pady=10,sticky="N")

        # ---------------------------------------------------------------------------------------
        # CTE Lab Image -------------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------
        #canvas = tk.Canvas(window, width=200, height=183)
        #canvas.grid(row=0, column=4, rowspan=2, columnspan=2, padx= 10, pady=10)
        camera = cv2.VideoCapture(1)

        camera.set(cv2.CAP_PROP_EXPOSURE, 100)

        lbl = tk.Label(window)
        lbl.grid(row=0, column=4, rowspan=2, columnspan=2, padx=10, pady=10)

        def update():
            ret, frame = camera.read()
            if not ret:
                messagebox.showerror(title='Imaging Failure', message="The Camera Couldn't Load, Click the Image to Retry")
                return

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            resize = cv2.resize(gray, (200, 183))
            image = Image.fromarray(resize)
            #cv2.imshow('temp', frame)
            fixed = ImageTk.PhotoImage(image)

            lbl.config(image=fixed)
            lbl.image = fixed
            lbl.bind('<Button-1>', EnlargedVideo)

            window.after(1, update)

        #image = image.resize((200, 200))
        #Rendition = ImageTk.PhotoImage(image)

        #image = Image.open(r"C:\Users\CTELab\Pictures\Screenshots\Screenshot 2023-06-21 091959.png")
        #image = image.resize((200, 200))
        #Rendition = ImageTk.PhotoImage(image)

        # Video = canvas.create_image(100, 137, image=Rendition)
        # canvas.tag_bind(Video, '<Button-1>', EnlargedVideo)

        # ---------------------------------------------------------------------------------------
        # Button Operation ----------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        QuitButton = tk.Button(window, bd=5, bg="#ff0000", text="Quit", height=3, width=15, command=quitApp)
        QuitButton.grid(column=4, row=3, columnspan=2, rowspan=2, padx=10, pady=10)

        SubmitButton = tk.Button(window, bd=5, bg="#90EE90", text="Cyclic", height=3, width=15, command=Cyclic)
        SubmitButton.grid(column=2, row=3, columnspan=1, rowspan=2, padx=10, pady=10)

        Zero = tk.Button(window, bd=5, bg="#E0B0FF", text="Home", height=3, width=15, command=Setzero)
        Zero.grid(column=0, row=3, columnspan=2, rowspan=2, padx=10, pady=10)

        Position = tk.Button(window, bd=5, bg="#90EE90", text="Move To Position", height=3, width=15,command=Position)
        Position.grid(column=3, row=3, columnspan=1, rowspan=2, padx=10, pady=10)

        # ---------------------------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------

        update()
        window.mainloop()
        camera.release()

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
