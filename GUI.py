import time
from guizero import App, Text, Slider

calibrationValue = 0
P = 0
I = 0
D = 0

def changeCalibrationValue(CalibrationSliderValue):
    global calibrationValue
    calibrationValue = str(CalibrationSliderValue)
def changePValue(PSliderValue):
    global P
    P = str(PSliderValue)
def changeIValue(ISliderValue):
    global I
    I = str(ISliderValue)
def changeDValue(DSliderValue):
    global D
    D = str(DSliderValue)
def counter():
    print("calibrationValue = ", calibrationValue)
    print("P = ", P)
    print("I = ", I)
    print("D = ", D)

app = App(title = "PID Control Setting")
text = Text(app, text = 1)
text.repeat(1000, counter)
welcomeMessage = Text(app, text = "PID Control Setting")

calibrationSlider = Slider(app, command = changeCalibrationValue, start=0, end=100)
PSlider = Slider(app, command = changePValue, start=0, end=100)
ISlider = Slider(app, command = changeIValue, start=0, end=100)
DSlider = Slider(app, command = changeDValue, start=0, end=100)
app.display()