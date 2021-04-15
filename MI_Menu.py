import PySimpleGUI as sg
from Serial_Comunication import *
import Get_Resources
sg.theme('DarkAmber')   # Add a touch of color
# All the stuff inside your window.
layoutMenu = [  [sg.Text('Projeto de Microcontroladores e Interfaces',justification='center',size=(50,1),font=('Arial',10))],
            [sg.Image('Resources\\UM_ENG.png',size=(400,200))],
            [sg.Button('Gráficos de Aceleração em Tempo Real',size=(50,1))],
            [sg.Button('BB8',size=(50,1))],
            [sg.Button('Tracker de Posição',size=(50,1))],
            [sg.Button('Sair',size=(50,1))],
            [sg.Text('Leander Reascos & Tiago Pereira',justification='center',size=(50,1))],
            [sg.Text('2021',justification='center',size=(50,1))] ]


# Create the Window
windowMain = sg.Window('Projeto de Microcontroladores e Interfaces', resizable=True).Layout(layoutMenu)
# Event Loop to process "events" and get the "values" of the inputs
COMPort = int(sg.popup_get_text('Insira o número da porta COM a utilizar para comunicação serial:'
                    , title='Projeto de Microcontroladores e Interfaces'
                    , default_text='3'))

serial_MI = Data_Visualization( 'COM'+str(COMPort), 10417)
isRun = serial_MI.conect_SerialPort()


program = ''

while isRun:
    event, values = windowMain.read()
    if event == sg.WIN_CLOSED or event == 'Sair':
        break
    if event == 'Gráficos de Aceleração em Tempo Real':
        print('Carregango Gráficos de Aceleração em Tempo Real...')
        program = 'RealTime'
        break
    if event == 'BB8':
        print('Carregango BB8...')
        program = 'BB8'
        break
    if event == 'Tracker de Posição':
        print('Carregango Tracker de Posição...')
        program = 'Tracker'
        break
windowMain.close()

if program == 'BB8':
    serial_MI.readSerialStart()
    serial_MI.create_scene(2000,900,0.5)
    serial_MI.control_enable()

elif program == 'RealTime':
    serial_MI.readSerialStart()
    serial_MI.create_plot()

elif program == 'Tracker':
    serial_MI.readSerialStart()
    serial_MI.create_scene(2000,900,0.1)
    serial_MI.tracker_enable()
