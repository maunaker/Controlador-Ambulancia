#//////////////////////////////////////////////////////////////////////
#//         Programado Y Desarrollado por : Mauricio J. Schonaker    //
#//                             Marzo 2022                           //
#//                        MAUNAKER@GMAIL.COM                        //
#//                Software para Hard RPI4 Ambulancia                //
#//////////////////////////////////////////////////////////////////////


from tkinter import*
import tkinter as tk
import RPi.GPIO as GPIO
import smbus
import time
from tkinter import messagebox as mb
import threading
import numpy as np

#///////////////variables globales ///////////////////////////////////
VBAT=13.5 #limite alternador
varluzpaciente=False
varluztecho=False
varluzarmario=False
vartoma121=False
varLatch12v1=False
varLatch12v2=False
varLatch220=False
vartoma122=False
vartoma220=False
varmaletines=False
varNivelH2O=False
varUSB=False
varReflector=False
varMotorOn=False
varExisten220=False
varTensiondebateria=0.0
contador=30#para salir del vehiculo
contador2=20
varLatch220inicial=False
varev1=False
varev2=False
nivel1=0.0
nivel1viejo=0.0
nivel2viejo=0.0
varTensiondebateriaviejo=0.0
nivel2=0.0
varNivelDesinfectante=False
umbraltubo=95#umbral 0-360 tubo lleno y vacio
varLatchEv1=False
varLatchEv2=False

#//////////////////////////sets GPIO///////////////////////
try:
    #GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4,GPIO.OUT,initial=GPIO.HIGH)#luzpaciente 4
    GPIO.setup(17,GPIO.OUT,initial=GPIO.HIGH)#luz techo
    GPIO.setup(27,GPIO.OUT,initial=GPIO.HIGH)#h2o
    GPIO.setup(22,GPIO.OUT,initial=GPIO.HIGH)#usb5v
    GPIO.setup(10,GPIO.OUT,initial=GPIO.HIGH)#toma12v1
    GPIO.setup(9,GPIO.OUT,initial=GPIO.HIGH)#toma12v2
    GPIO.setup(11,GPIO.OUT,initial=GPIO.HIGH)#220 inversor
    GPIO.setup(5,GPIO.OUT,initial=GPIO.HIGH)#ev1
    GPIO.setup(6,GPIO.OUT,initial=GPIO.HIGH)#ev2
    GPIO.setup(13,GPIO.OUT,initial=GPIO.HIGH)#maletines
    GPIO.setup(26,GPIO.OUT,initial=GPIO.HIGH)#luz armario
    GPIO.setup(19,GPIO.OUT,initial=GPIO.HIGH)#reflector
    GPIO.setup(0,GPIO.OUT,initial=GPIO.HIGH)#vaporizador
    
    GPIO.setup(23,GPIO.IN)#sensor 220 presente
    GPIO.setup(24,GPIO.IN)#sensor nivel h2o
    GPIO.setup(25,GPIO.IN)#sensor nivel desinfectante
    GPIO.setup(20,GPIO.IN)#sensor 12v1 presente
    GPIO.setup(21,GPIO.IN)#sensor 12v2 presente
    GPIO.setup(7,GPIO.IN)#sensor ev1 encendida
    GPIO.setup(8,GPIO.IN)#sensor ev2 encendida
except:
    GPIO.cleanup(4)
    GPIO.setup(4,GPIO.IN)
    GPIO.setup(4,GPIO.OUT)
    GPIO.cleanup(0)
    GPIO.setup(0,GPIO.IN)
    GPIO.setup(0,GPIO.OUT)
    GPIO.cleanup(19)
    GPIO.setup(19,GPIO.IN)
    GPIO.setup(19,GPIO.OUT)
    GPIO.cleanup(17)
    GPIO.setup(17,GPIO.IN)
    GPIO.setup(17,GPIO.OUT)
    GPIO.cleanup(27)
    GPIO.setup(27,GPIO.IN)
    GPIO.setup(27,GPIO.OUT)
    GPIO.cleanup(22)
    GPIO.setup(22,GPIO.IN)
    GPIO.setup(22,GPIO.OUT)
    GPIO.cleanup(10)
    GPIO.setup(10,GPIO.IN)
    GPIO.setup(10,GPIO.OUT)
    GPIO.cleanup(9)
    GPIO.setup(9,GPIO.IN)
    GPIO.setup(9,GPIO.OUT)
    GPIO.cleanup(11)
    GPIO.setup(11,GPIO.IN)
    GPIO.setup(11,GPIO.OUT)
    GPIO.cleanup(5)
    GPIO.setup(5,GPIO.IN)
    GPIO.setup(5,GPIO.OUT)
    GPIO.cleanup(6)
    GPIO.setup(6,GPIO.IN)
    GPIO.setup(6,GPIO.OUT)
    GPIO.cleanup(13)
    GPIO.setup(13,GPIO.IN)
    GPIO.setup(13,GPIO.OUT)
    GPIO.cleanup(26)
    GPIO.setup(26,GPIO.IN)
    GPIO.setup(26,GPIO.OUT)

#///////////control ev1 y ev2 para entradas de o2///////////////////////////////////

def controlEV():

    global varev1
    global varev2
    global varLatchEv1
    global varLatchEv2
    
    while True:
        if GPIO.input(7)==False:#True:
            varLatchEv1=True
        else:
            varLatchEv1=False
            
        if  GPIO.input(8)==False:#True:
            varLatchEv2=True
        else:
            varLatchEv2=False
        
        if (nivel1>umbraltubo):#si el tubo 1 tiene o2
            if varLatchEv1==False:#ev1 apagada ->la enciende

                GPIO.output(5,GPIO.LOW)#HIGH)#pulso al latch
                time.sleep(0.1)
                GPIO.output(5,GPIO.HIGH)#LOW)
                
            if varLatchEv2==True:#ev2 encendida ->la apaga

                GPIO.output(6,GPIO.LOW)#HIGH)#pulso al latch
                time.sleep(0.1)
                GPIO.output(6,GPIO.HIGH)#LOW)

            varev1=True#para actualizar gui
            varev2=False

        else:#tubo 1 no tiene o2
            if (nivel2>umbraltubo):#si el tubo 2 tiene o2
                if varLatchEv2==False:#ev2 apagada ->la enciende

                    GPIO.output(6,GPIO.LOW)#HIGH)#pulso al latch
                    time.sleep(0.1)
                    GPIO.output(6,GPIO.HIGH)#LOW)

                if varLatchEv1==True:#ev1 encendida ->la apaga

                    GPIO.output(5,GPIO.LOW)#HIGH)#pulso al latch
                    time.sleep(0.1)
                    GPIO.output(5,GPIO.HIGH)#LOW)

                varev1=False#para actualizar gui
                varev2=True
                    
            else:#ningun tubo tiene o2// usa remanente de los 2 tubos//conecta los 2
                if varLatchEv1==False:#ev1 apagada ->la enciende

                    GPIO.output(5,GPIO.LOW)#HIGH)#pulso al latch
                    time.sleep(0.1)
                    GPIO.output(5,GPIO.HIGH)#LOW)
                    time.sleep(0.5)

                if varLatchEv2==False:#ev2 apagada ->la enciende

                    GPIO.output(6,GPIO.LOW)#HIGH)#pulso al latch
                    time.sleep(0.1)
                    GPIO.output(6,GPIO.HIGH)#LOW)
                    time.sleep(0.5)

                varev1=False
                varev2=False
       
        

#/////////////////lectura valores ANalogicos PCF8951 //////////////////////////////////
bus=smbus.SMBus(1)
def LeePcf8951():
    global nivel1
    global nivel2
    global varTensiondebateria
    while True:
        bus.write_byte(0x48,0x40)#AN0 pin PCF8591 senial vbat12v
        varTensiondebateria=round(bus.read_byte(0x48)*20/255,2) #0-20v divisor x4
        time.sleep(0.1)
        bus.write_byte(0x48,0x41)#AN1 pin PCF8591 senial tubo 1
        nivel1= bus.read_byte(0x48)*360/255#*1.4115
        time.sleep(0.1)
        bus.write_byte(0x48, 0x42)#AN2 pin senial tubo2
        nivel2= bus.read_byte(0x48)*360/255#*1.4115
        time.sleep(0.1)
    
    
#//////////////////////////////////funciones/////////////////////////////////////////////////////////
def cmdboton1():#luz paciente
    if (varluzpaciente==False):
        GPIO.output(4,GPIO.LOW)        
    else:
        GPIO.output(4,GPIO.HIGH)        
         
def cmdboton3():#luz techo
    if (varluztecho==False):
        GPIO.output(17,GPIO.LOW)       
    else:
        GPIO.output(17,GPIO.HIGH)
        
def cmdboton5():#luz armario
    if (varluzarmario==False):
        GPIO.output(26,GPIO.LOW)      
    else:
        GPIO.output(26,GPIO.HIGH)
        
def cmdboton4():#reflector  
    if (varReflector==False):
        GPIO.output(19,GPIO.LOW)      
    else:
        GPIO.output(19,GPIO.HIGH)
        
def cmdboton11(): # maletines 
    if (varmaletines==False):
        GPIO.output(13,GPIO.LOW)      
    else:
        GPIO.output(13,GPIO.HIGH)
        
def cmdboton8(): # hab h2o
    if (varH2O==False):
        GPIO.output(27,GPIO.LOW)      
    else:
        GPIO.output(27,GPIO.HIGH)
        
def cmdboton10(): # hab 5vusb    
    if (varUSB==False):  
        GPIO.output(22,GPIO.LOW)         
    else:  
        GPIO.output(22,GPIO.HIGH)
         
def cmdboton9(): # hab 12v1 latcheado
    if (varLatch12v1==True):
        msgbx=mb.askquestion("ADVERTENCIA","Los 12 Volts forman parte del soporte vital. Desea desactivarlo?")
        if msgbx=='yes':
            GPIO.output(10,GPIO.LOW)#HIGH)#pulso al latch pic12f675
            time.sleep(0.1)
            GPIO.output(10,GPIO.HIGH)#LOW)
    else:
        GPIO.output(10,GPIO.LOW)#HIGH)
        time.sleep(0.1)
        GPIO.output(10,GPIO.HIGH)#LOW)
              
def cmdboton7(): # hab 12v2 latcheado
    if (varLatch12v2==True):
        msgbx=mb.askquestion("ADVERTENCIA","Los 12 Volts forman parte del soporte vital. Desea desactivarlo?")
        if msgbx=='yes':
            GPIO.output(9,GPIO.LOW)#HIGH)#pulso al latch pic12f675
            time.sleep(0.1)
            GPIO.output(9,GPIO.HIGH)#LOW)
    else:
        GPIO.output(9,GPIO.LOW)#HIGH)
        time.sleep(0.1)
        GPIO.output(9,GPIO.HIGH)#LOW)
        
def cmdboton6(): # hab 220v latcheado
    if (varLatch220==True):
        msgbx=mb.askquestion("ADVERTENCIA","Los 220 Volts forman parte del soporte vital. Desea desactivarlo?")
        if msgbx=='yes':
            GPIO.output(11,GPIO.LOW)#HIGH)#pulso al latch pic12f675
            time.sleep(0.1)
            GPIO.output(11,GPIO.HIGH)#LOW)
    else:
        GPIO.output(11,GPIO.LOW)#HIGH)
        time.sleep(0.1)
        GPIO.output(11,GPIO.HIGH)#LOW)
        
        
#/////////////////////ventana secundaria de desinfeccion////////////////////
        
def createNewWindow():
    newWindow = tk.Toplevel(raiz)
    newWindow.title("Proceso de desinfeccion automatico.")
    newWindow.config(cursor='none')
    newWindow._bgcolor = 'red2'  # X11 color: 'gray85'
    newWindow._fgcolor = 'red2'  # X11 color: 'black'
    newWindow._compcolor = 'red2' # X11 color: 'gray85'
    newWindow._ana1color = 'red2' # X11 color: 'gray85'
    newWindow._ana2color = 'red2' # Closest X11 color: 'gray92'
    newWindow.geometry("1024x450+238+169")
    newWindow.minsize(120, 1)
    newWindow.maxsize(1540, 845)
    newWindow.resizable(1,  1)
    newWindow.configure(background="red2")
    newWindow.configure(highlightbackground="red2")
    newWindow.configure(highlightcolor="red2")
    #newframe=Frame(newWindow) #crea el frame en la ventana raiz
    newWindow.attributes('-fullscreen', True) 

    def iniciar_proceso_desinfeccion():
        varLatch220inicial=varLatch220
        Label24.config(text=f"Iniciando el proceso...")
        #ver si hay 220 marcha y desinfectante
        if  varNivelDesinfectante==True:#hay desinfectante
            if varLatch220==False:#no hay 220
                GPIO.output(11,GPIO.LOW)#HIGH)#gpio11 220
                time.sleep(0.1)
                GPIO.output(11,GPIO.HIGH)#LOW)
                time.sleep(1)
                if varLatch220==False:
                    Label25.config(text=f"FALLA! No se pudieron activar los 220 VAC.")
                    Button21.configure(state='active')
            else:
                if varMotorOn==False:
                    Label25.config(text=f"FALLA! El motor del vehiculo no esta encendido.") 
                    Button21.configure(state='active')
                else:
                    GPIO.output(0,GPIO.LOW)#HIGH)#gpio0 vaporizador a 220
                    newWindow.after(0,cuenta_desinfeccion) 
        else:
            Label25.config(text=f"FALLA! No hay liquido desinfectante.")
            Button21.configure(state='active')

    def cuenta_regresiva():
        global contador 
        Button21.configure(state='disabled')
        Label24.config(text=f"El proceso comenzara en:  {contador} segundos...")
        Label25.config(text=f"Abandone y cierre el vehiculo!.")
        contador-=1
        if contador>-1:
            newWindow.after(1000, cuenta_regresiva)
        else:
            newWindow.after(0,iniciar_proceso_desinfeccion)
             
    def cuenta_desinfeccion():
        global contador2
        Button21.configure(state='disabled')
        Label24.config(text=f"Desinfectando: {contador2} segundos...")
        contador2-=1
        if contador2>-1:
            newWindow.after(1000, cuenta_desinfeccion)
        else:
            Label24.config(text=f"El proceso se ha completado, ventile el vehiculo.")
            #desactivar inversor y calentaor
            GPIO.output(0,GPIO.HIGH)#LOW)#gpio0 vaporizador a 220
            if varLatch220inicial==False:#si los 220 iniciales estaban apagados apaga inversor
                GPIO.output(11,GPIO.LOW)#HIGH)#latch
                time.sleep(0.1)
                GPIO.output(11,GPIO.HIGH)#LOW)
                contador2=contador2
            Button21.configure(state='enabled')

    Button21 =tk.Button(newWindow, text = "INICIAR PROCESO")
    Button21.place(relx=0.791, rely=0.517, height=84, width=207)
    Button21.configure(activebackground="red")
    Button21.configure(activeforeground="#000000")
    Button21.configure(background="gold")
    Button21.configure(compound='left')
    Button21.configure(disabledforeground="#a3a3a3")
    Button21.configure(foreground="black")
    Button21.configure(highlightbackground="#d9d9d9")
    Button21.configure(highlightcolor="black")
    Button21.configure(pady="0")
    Button21.configure(command=cuenta_regresiva)

    Button22 =tk.Button(newWindow, text = "CANCELAR")
    Button22.place(relx=0.791, rely=0.750, height=84, width=207)
    Button22.configure(activebackground="red")
    Button22.configure(activeforeground="#000000")
    Button22.configure(background="RoyalBlue3")
    Button22.configure(compound='left')
    Button22.configure(disabledforeground="#a3a3a3")
    Button22.configure(foreground="black")
    Button22.configure(highlightbackground="#d9d9d9")
    Button22.configure(highlightcolor="black")
    Button22.configure(pady="0")
    Button22.configure(command=newWindow.destroy)
    
    Label20 = tk.Label(newWindow, text = "ATENCION!: Para este proceso debera desalojar el vehiculo.",font=('Arial',15,'bold'))
    Label20.place(relx=0.01, rely=0.017, height=41, width=1000)
    Label20.configure(activebackground="#f9f9f9")
    Label20.configure(activeforeground="black")
    Label20.configure(anchor='w')
    Label20.configure(background="red2")
    Label20.configure(compound='left')
    Label20.configure(disabledforeground="#a3a3a3")
    Label20.configure(foreground="#000000")
    Label20.configure(highlightbackground="#d9d9d9")
    Label20.configure(highlightcolor="white")

    Label22 = tk.Label(newWindow, text = "El motor se debera estar funcionando y el vehiculo detenido.",font=('Arial',15,'bold'))
    Label22.place(relx=0.13, rely=0.117, height=41, width=1000)
    Label22.configure(activebackground="#f9f9f9")
    Label22.configure(activeforeground="black")
    Label22.configure(anchor='w')
    Label22.configure(background="red2")
    Label22.configure(compound='left')
    Label22.configure(disabledforeground="#a3a3a3")
    Label22.configure(foreground="#000000")
    Label22.configure(highlightbackground="#d9d9d9")
    Label22.configure(highlightcolor="white")
   
    Label23 = tk.Label(newWindow, text = "Si procede con la desinfeccion pulse INICIAR PROCESO, de lo contrario CANCELAR",font=('Arial',15,'bold'))
    Label23.place(relx=0.13, rely=0.217, height=41, width=1000)
    Label23.configure(activebackground="#f9f9f9")
    Label23.configure(activeforeground="black")
    Label23.configure(anchor='w')
    Label23.configure(background="red2")
    Label23.configure(compound='left')
    Label23.configure(disabledforeground="#a3a3a3")
    Label23.configure(foreground="#000000")
    Label23.configure(highlightbackground="#d9d9d9")
    Label23.configure(highlightcolor="white")
    
    Label24 = tk.Label(newWindow, text = "El proceso se iniciara en:"+" de lo contrario CANCELAR",font=('Arial',20,'bold'))
    Label24.place(relx=0.01, rely=0.417, height=41, width=1000)
    Label24.configure(activebackground="#f9f9f9")
    Label24.configure(activeforeground="black")
    Label24.configure(anchor='w')
    Label24.configure(background="red2")
    Label24.configure(compound='left')
    Label24.configure(disabledforeground="#a3a3a3")
    Label24.configure(foreground="#000000")
    Label24.configure(highlightbackground="#d9d9d9")
    Label24.configure(highlightcolor="white")

    Label25 = tk.Label(newWindow, text = "",font=('Arial',20,'bold'))
    Label25.place(relx=0.01, rely=0.647, height=41, width=800)
    Label25.configure(activebackground="#f9f9f9")
    Label25.configure(activeforeground="yellow")
    Label25.configure(anchor='w')
    Label25.configure(background="red2")
    Label25.configure(compound='left')
    Label25.configure(disabledforeground="#a3a3a3")
    Label25.configure(foreground="#000000")
    Label25.configure(highlightbackground="#d9d9d9")
    Label25.configure(highlightcolor="white")
        
# /////////////////////////estados, sensores ,evs,etc////////////////

def estados():
    #/////estado boton 12v1 y 12v2/////////////
    global varLatch12v1
    global varluzpaciente
    global varMotorOn
    global varLatch12v2
    global varluzarmario
    global varReflector
    global varluztecho
    global varNivelH2O
    global varmaletines
    global varH2O
    global varLatch220
    global varUSB
    
    while True:
        
        varluztecho=not GPIO.input(17)
        if varluztecho==True:
            Button3.configure(background="green3")
        else:
            Button3.configure(background="Gray70")
        
        varluzarmario=not GPIO.input(26)
        if varluzarmario==True:
            Button5.configure(background="SpringGreen2")
        else:
            Button5.configure(background="Gray70")
        
        varReflector=not GPIO.input(19)
        
        if varReflector==True:
            Button4.configure(background="SpringGreen3")
        else:
            Button4.configure(background="Gray70")
            
        varmaletines=not GPIO.input(13)
        if varmaletines==True:
            Button11.configure(background="SpringGreen4")
        else:
            Button11.configure(background="Gray70")
            
        varluzpaciente=not GPIO.input(4)
        if varluzpaciente==True:
            Button1.configure(background="green2")
        else:
            Button1.configure(background="Gray70")
       
        varH2O=not GPIO.input(27)
        if varH2O==True:
            Button8.configure(background="Turquoise1")
        else:
            Button8.configure(background="Gray70")
        
        varUSB=not GPIO.input(22)
        if varUSB==True:
            Button10.configure(background="orange")
        else:
            Button10.configure(background="Gray70")
        
        if (GPIO.input(20)==False):#12v1 sensor
            varLatch12v1==True
            Button9.configure(background="yellow")
        else:
            Button9.configure(background="Gray70")
            varLatch12v1==False
            
        if (GPIO.input(21)==False):#True):#12v2 sensor   
            varLatch12v2==True
            Button7.configure(background="yellow")
        else:
            Button7.configure(background="Gray70")
            varLatch12v2==False
         
         
       #/////estado boton 220/////////////   
       
        if (GPIO.input(23)==False):#12v1 sensor invertda!
            varLatch220==True
            Button6.configure(background="magenta2")
            Label2.configure(text='''220 VAC: SI''',foreground="green2")
        else:
            Button6.configure(background="Gray70")
            varLatch220==False 
            Label2.configure(text='''220 VAC: NO''',foreground="orange red")
         
        #///tension bateria////
        
        Label1.configure(text='''VDC: '''+ np.str(varTensiondebateria))

        #//////marcha motor////////
        
        if varTensiondebateria >VBAT:
            Label3.configure(text='''Motor:ENC.''',foreground="green2")
            varMotorOn=True
        else:
            Label3.configure(text='''Motor:APAG.''',foreground="orange red")
            varMotorOn=False
           
        #///////////////nivel h2o////////////////

        if (GPIO.input(24)==False):#True):
            varNivelH2O=True    
            Label4.configure(text='''Nivel H2O: OK''',foreground="green2")     
        else:    
            Label4.configure(text='''Nivel H2O: X''',foreground="orange red")
            varNivelH2O=False 

        #///////////////nivel desinf////////////////

        global varNivelDesinfectante    
        if (GPIO.input(25)==False):
            varNivelDesinfectante=True
            Label5.configure(text='''Desinfectante:OK''',foreground="green2")    
        else:
            varNivelDesinfectante=False
            Label5.configure(text='''Desinfectante:X''',foreground="orange red")
     
    #/////////////////////tubo1/y/tubo2///////////////////////////////////////////////

        if (nivel1>umbraltubo): 
            Label6.configure(text='''TuboO2-1:OK''',foreground="green2")
        else:  
            Label6.configure(text='''TuboO2-1:VACIO''',foreground="orange red")
            Label10.configure(text='''Tubo 1 SIN OXIGENO!''')
        if (nivel2>umbraltubo):
            Label7.configure(text='''TuboO2-2:OK''',foreground="green2") 
        else: 
            Label7.configure(text='''TuboO2-2:VACIO''',foreground="orange red")
            Label10.configure(text='''Tubo 2 SIN OXIGENO!''')

        #/////actualizacion label 9 y 10//////////////////////////
        
        if (varev1==True and varev2==False):
            Label9.configure(text='''Usando Oxigeno del Tubo 1''')
        if (varev2==True and varev1==False):
            Label9.configure(text='''Usando Oxigeno del Tubo 2''')
        if (varev2==True and varev1==True):
            Label9.configure(text='''ATENCION:NO HAY OXIGENO''')
            Label10.configure(text='''USANDO EL REMANENTE''')
        if (varev2==False )and( varev1==False):
            Label9.configure(text='''FALLA ELECTRONICA DE OXIGENO''')
            Label10.configure(text='''PASE AL MODO MANUAL''')
        
#//////////////////////////////////////////////////////////////////////////////////////////////////   
#//////////////////////////interface////////////////////////////
raiz=tk.Tk()
raiz.title("CPU V1.0")
raiz.config(cursor='none')
raiz._bgcolor = '#d9d9d9'  # X11 color: 'gray85'
raiz._fgcolor = '#000000'  # X11 color: 'black'
raiz._compcolor = '#d9d9d9' # X11 color: 'gray85'
raiz._ana1color = '#d9d9d9' # X11 color: 'gray85'
raiz._ana2color = '#ececec' # Closest X11 color: 'gray92'
raiz.geometry("1024x450+238+169")
raiz.minsize(120, 1)
raiz.maxsize(1540, 845)
raiz.resizable(1,  1)
raiz.configure(background="#d9d9d9")
raiz.configure(highlightbackground="#d9d9d9")
raiz.configure(highlightcolor="black")
raiz.attributes('-fullscreen', True)  

miframe=Frame() #crea el frame en la ventana raiz
miframe.config(bg="gray")
miframe.config(width="1200",height="600")
miframe.place(relx=0.0, rely=0.0, relheight=1.333, relwidth=1.0)
miframe.configure(background="black")
miframe.configure(highlightbackground="black")
miframe.configure(highlightcolor="black")

Button1 =Button()
Button1.place(relx=0.791, rely=0.017, height=64, width=207)
Button1.configure(background="Gray70")
Button1.configure(foreground="#000000")
Button1.configure(text='''LUZ PACIENTE''')
Button1.configure(command=cmdboton1)

Button2 = Button()
Button2.place(relx=0.791, rely=0.850, height=64, width=207)
Button2.configure(background="Gray70")
Button2.configure(highlightbackground="#d9d9d9")
Button2.configure(text='''DESINFECTAR''')
Button2.configure(command=createNewWindow)

Button3 = Button()
Button3.place(relx=0.791, rely=0.160, height=64, width=207)
Button3.configure(background="Gray70")
Button3.configure(highlightbackground="#d9d9d9")
Button3.configure(text='''LUZ TECHO''')
Button3.configure(command=cmdboton3)

Button4 = Button()
Button4.place(relx=0.791, rely=0.457, height=64, width=207)
Button4.configure(background="Gray70")
Button4.configure(highlightbackground="#d9d9d9")
Button4.configure(text='''REFLECTOR TRASERO''')
Button4.configure(command=cmdboton4)

Button5 = Button()
Button5.place(relx=0.791, rely=0.307, height=64, width=207)
Button5.configure(background="Gray70")
Button5.configure(highlightbackground="#d9d9d9")
Button5.configure(text='''LUZ ARMARIO''')
Button5.configure(command=cmdboton5)

Button11 = Button()
Button11.place(relx=0.791, rely=0.610, height=64, width=207)
Button11.configure(background="Gray70")
Button11.configure(highlightbackground="#d9d9d9")
Button11.configure(highlightcolor="black")
Button11.configure(text='''HAB. MALETINES''')
Button11.configure(command=cmdboton11)

Button6 = Button()
Button6.place(relx=0.322, rely=0.850, height=64, width=147)
Button6.configure(background="Gray70")
Button6.configure(highlightbackground="#d9d9d9")
Button6.configure(text='''HAB. 200VCA''')
Button6.configure(command=cmdboton6)

Button7 = Button()
Button7.place(relx=0.166, rely=0.850, height=64, width=147)
Button7.configure(background="Gray70")
Button7.configure(highlightbackground="#d9d9d9")
Button7.configure(text='''HAB. 12VDC II''')
Button7.configure(command=cmdboton7)

Button8 = Button()
Button8.place(relx=0.479, rely=0.850, height=64, width=147)
Button8.configure(background="Gray70")
Button8.configure(highlightbackground="#d9d9d9")
Button8.configure(text='''HAB. H2O''')
Button8.configure(command=cmdboton8)

Button9 = Button()
Button9.place(relx=0.01, rely=0.850, height=64, width=147)
Button9.configure(background="Gray70")
Button9.configure(highlightbackground="#d9d9d9")
Button9.configure(text='''HAB. 12VDC I''')
Button9.configure(command=cmdboton9)

Button10 = Button()
Button10.place(relx=0.635, rely=0.850, height=64, width=147)
Button10.configure(text='''HAB. 5V USB''')
Button10.configure(command=cmdboton10)

Label1 = Label()
Label1.place(relx=0.01, rely=0.017, height=40, width=207)
Label1.configure(activebackground="#f9f9f9")
Label1.configure(activeforeground="green2")
Label1.configure(anchor='w')
Label1.configure(background="midnight blue")
Label1.configure(compound='left')
Label1.configure(disabledforeground="#a3a3a3")
Label1.configure(foreground="green2")
Label1.configure(highlightbackground="#d9d9d9")
Label1.configure(highlightcolor="black")
Label1.configure(text='''VDC: ''')
Label1.configure(font=("Arial",20))
Label1.configure(relief='sunken',borderwidth=3)


Label2 = Label()
Label2.place(relx=0.01, rely=0.12, height=40, width=207)
Label2.configure(activebackground="#f9f9f9")
Label2.configure(activeforeground="green2")
Label2.configure(anchor='w')
Label2.configure(background="midnight blue")
Label2.configure(compound='left')
Label2.configure(disabledforeground="#a3a3a3")
Label2.configure(foreground="green2")
Label2.configure(highlightbackground="#d9d9d9")
Label2.configure(highlightcolor="black")
Label2.configure(text='''220 VAC: ''')
Label2.configure(font=("Arial",20))
Label2.configure(relief='sunken',borderwidth=3)

Label3 = Label()
Label3.place(relx=0.01, rely=0.225, height=40, width=207)
Label3.configure(activebackground="#f9f9f9")
Label3.configure(activeforeground="green2")
Label3.configure(anchor='w')
Label3.configure(background="midnight blue")
Label3.configure(compound='left')
Label3.configure(disabledforeground="#a3a3a3")
Label3.configure(foreground="green2")
Label3.configure(highlightbackground="#d9d9d9")
Label3.configure(highlightcolor="black")
Label3.configure(text='''Motor: ''')
Label3.configure(font=("Arial",20))
Label3.configure(relief='sunken',borderwidth=3)

Label4 = Label()
Label4.place(relx=0.01, rely=0.330, height=40, width=207)
Label4.configure(activebackground="#f9f9f9")
Label4.configure(activeforeground="green2")
Label4.configure(anchor='w')
Label4.configure(background="midnight blue")
Label4.configure(compound='left')
Label4.configure(disabledforeground="#a3a3a3")
Label4.configure(foreground="green2")
Label4.configure(highlightbackground="#d9d9d9")
Label4.configure(highlightcolor="black")
Label4.configure(text='''Nivel H2O: ''')
Label4.configure(font=("Arial",20))
Label4.configure(relief='sunken',borderwidth=3)

Label5 = Label()
Label5.place(relx=0.01, rely=0.435, height=40, width=207)
Label5.configure(activebackground="#f9f9f9")
Label5.configure(activeforeground="green2")
Label5.configure(anchor='w')
Label5.configure(background="midnight blue")
Label5.configure(compound='left')
Label5.configure(disabledforeground="#a3a3a3")
Label5.configure(foreground="green2")
Label5.configure(highlightbackground="#d9d9d9")
Label5.configure(highlightcolor="black")
Label5.configure(text='''Desinfectante: ''')
Label5.configure(font=("Arial",20))
Label5.configure(relief='sunken',borderwidth=3)

Label6 = Label()
Label6.place(relx=0.01, rely=0.540, height=40, width=207)
Label6.configure(activebackground="#f9f9f9")
Label6.configure(activeforeground="green2")
Label6.configure(anchor='w')
Label6.configure(background="midnight blue")
Label6.configure(compound='left')
Label6.configure(disabledforeground="#a3a3a3")
Label6.configure(foreground="green2")
Label6.configure(highlightbackground="#d9d9d9")
Label6.configure(highlightcolor="black")
Label6.configure(text='''OxigenoTb1:''')
Label6.configure(font=("Arial",20))
Label6.configure(relief='sunken',borderwidth=3)

Label7 = Label()
Label7.place(relx=0.01, rely=0.645, height=40, width=207)
Label7.configure(activebackground="#f9f9f9")
Label7.configure(activeforeground="green2")
Label7.configure(anchor='w')
Label7.configure(background="midnight blue")
Label7.configure(compound='left')
Label7.configure(disabledforeground="#a3a3a3")
Label7.configure(foreground="green2")
Label7.configure(highlightbackground="#d9d9d9")
Label7.configure(highlightcolor="black")
Label7.configure(text='''OxigenoTb2: ''')
Label7.configure(font=("Arial",20))
Label7.configure(relief='sunken',borderwidth=3)

Label8 = Label()
Label8.place(relx=0.01, rely=0.750, height=40, width=207)
Label8.configure(activebackground="#f9f9f9")
Label8.configure(activeforeground="green2")
Label8.configure(anchor='w')
Label8.configure(background="midnight blue")
Label8.configure(compound='left')
Label8.configure(disabledforeground="#a3a3a3")
Label8.configure(foreground="green2")
Label8.configure(highlightbackground="#d9d9d9")
Label8.configure(highlightcolor="black")
Label8.configure(text='''O2-Usando: ''')
Label8.configure(font=("Arial",20))
Label8.configure(relief='sunken',borderwidth=3)

Label9 = Label()
Label9.place(relx=0.25, rely=0.5, height=40, width=510)
Label9.configure(activebackground="#f9f9f9")
Label9.configure(activeforeground="green2")
Label9.configure(anchor='w')
Label9.configure(background="gold")
Label9.configure(compound='left')
Label9.configure(disabledforeground="#a3a3a3")
Label9.configure(foreground="black")
Label9.configure(highlightbackground="#d9d9d9")
Label9.configure(highlightcolor="black")
Label9.configure(text='''---''')
Label9.configure(font=("Arial",20))
Label9.configure(relief='sunken',borderwidth=3)

Label10 = Label()
Label10.place(relx=0.25, rely=0.59, height=40, width=510)
Label10.configure(activebackground="#f9f9f9")
Label10.configure(activeforeground="green2")
Label10.configure(anchor='w')
Label10.configure(background="gold")
Label10.configure(compound='left')
Label10.configure(disabledforeground="#a3a3a3")
Label10.configure(foreground="black")
Label10.configure(highlightbackground="#d9d9d9")
Label10.configure(highlightcolor="black")
Label10.configure(text='''---''')
Label10.configure(font=("Arial",20))
Label10.configure(relief='sunken',borderwidth=3)

#/*//////////////////////////////////////////////progress tubos/////////////////////////////////////////////////////////*/

canvas=Canvas(miframe,bg='black',width=200,height=200,bd=0, highlightthickness=0, relief='ridge')
canvas.place(relx=0.283,rely=0.017,relwidth=0.49,relheight=0.405)

Xo=10 #posicion en x esquina sup izq

Yo=20 #posicion y esquina sup izq

Dx=220 #tamanio de circulos

Dy=Dx

Cx=Xo+Dx/2

Cy=Yo+Dy/2

X1=Xo+Dx

Y1=Yo+Dy

Sep=230

def progressbar():# proceso super lento
    
        canvas.delete("all")
     
        for i in range(np.int(nivel1)):
            canvas.create_line(Cx,Cy,Cx+Dx/2*np.cos(np.radians(i)),Cy-Dy/2*np.sin(np.radians(i)),fill='deep sky blue',width=20)
        
        canvas.create_line(Cx,Cy,Cx+Dx/2*np.cos(np.radians(0)),Cy-Dy/2*np.sin(np.radians(0)),fill='green2',width=20)
        
        canvas.create_oval(Xo,Yo,X1,Y1,fill='',outline='dark violet',width=6)
        
        canvas.create_oval(Xo+30,Yo+30,X1-30,Y1-30,fill='gray22',outline='dark violet',width=6)

        texto1=np.str(np.int(nivel1/3.6))+'%'
        
        canvas.create_text(Cx,Cy-10,text=texto1,font=('Arial',30,'bold'),fill='deep sky blue') 

        #///////el tubo2//////
        

        for i in range(np.int(nivel2)):
            canvas.create_line(Cx+Sep,Cy,Cx+Sep+Dx/2*np.cos(np.radians(i)),Cy-Dy/2*np.sin(np.radians(i)),fill='deep sky blue',width=20)
        
        canvas.create_line(Cx+Sep,Cy,Cx+Sep+Dx/2*np.cos(np.radians(0)),Cy-Dy/2*np.sin(np.radians(0)),fill='green2',width=20)
        
        canvas.create_oval(Xo+Sep,Yo,X1+Sep,Y1,fill='',outline='dark violet',width=6)
        
        canvas.create_oval(Xo+Sep+30,Yo+30,X1+Sep-30,Y1-30,fill='gray22',outline='dark violet',width=6)

        texto2=np.str(np.int(nivel2/3.6))+'%'

        canvas.create_text(Cx+Sep,Cy-10,text=texto2,font=('Arial',30,'bold'),fill='deep sky blue')
        
                
        canvas.create_text(Cx+Sep,Cy-10,text=texto2,font=('Arial',30,'bold'),fill='deep sky blue')
        
        
        canvas.create_text(Cx+Sep,Cy+25,text='TUBO II',font=('Cambria Math',15,'bold'),fill='white')
        
        canvas.create_text(Cx+Sep,Cy+50,text='O2',font=('Freestyle Script',20,'bold'),fill='orange')
        
        canvas.create_text(Cx,Cy+25,text='TUBO I',font=('Cambria Math',15,'bold'),fill='white')
        
        canvas.create_text(Cx,Cy+50,text='O2',font=('Freestyle Script',20,'bold'),fill='orange')

        raiz.after(2000,progressbar)#refresco cada 2 segundos

#///////////////////////////////////////main//////////////////////////////////////////
raiz.after(0,progressbar)# en thread dibuja super lento
hilo1=threading.Thread(target=LeePcf8951).start()
hilo2=threading.Thread(target=controlEV).start()
hilo3=threading.Thread(target=estados).start()
  
raiz.mainloop()


