############## Importar modulos #####################
from pyArduino import *

from tkinter import *
from PIL import Image, ImageTk #pip install pil

import cv2
import numpy as np
import sys

def toggle():
    btn.config(text=btnVar.get())
  
def onClossing():
    arduino.sendData([0,0])
    arduino.close()
    root.quit()         #Salir del bucle de eventos.
    cap.release()       #Cerrar camara
    print("Ip Cam Disconected")
    root.destroy()      #Destruye la ventana creada
    
    
def thresholdValue(int):
    umbralValue.set(slider.get())
    
def objectDetection(rawImage):
    
    
    kernel = np.ones((10,10),np.uint8) # Nucleo
    isObject = False     # Verdadero si encuentra un objeto
    cx,cy = 0,0          #centroide (x), centroide (y)
    
    minArea = 500  # Area minima para considerar que es un objeto

    ################# Procesamiento de la Imagen ################
    
    gray = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)
    t,binary = cv2.threshold(gray, umbralValue.get(), 255, cv2.THRESH_BINARY_INV)
    opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    ################# Segmentacion de la Imagen ################
    contours,_ = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        momentos = cv2.moments(cnt)
        area = momentos['m00']
        if (area>minArea):
            cx = int(momentos['m10']/momentos['m00'])     
            cy = int(momentos['m01']/momentos['m00'])
            isObject = True
            
    return isObject,binary,cx,cy
    
    
def callback():

        ################## Adquisición de la Imagen ############
        
        cap.open(url) # Antes de capturar el frame abrimos la url
        ret, frame = cap.read() # Leer Frame
        

        if ret:
            
            uRef = 0
            wRef = 0
                
            
            isObject,binary,cx,cy = objectDetection(frame)
            
            cv2.circle(frame,(cx,cy),10, (0,0,255), -1)
            cv2.circle(frame,(cxd,cyd),10, (0,255,0), -1)

            if isObject:
                
                hx = frame.shape[1]/2-cx
                
                
                hxe  = hxd-hx
                

                K = 0.0035
                
                uRef = 0.2
                wRef = -K*hxe

            else:
                uRef = 0
                wRef = 0

            if btnVar.get() == 'Start':
                arduino.sendData([uRef,wRef])
            else:
                arduino.sendData([0,0])
            
            
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    
            img = Image.fromarray(img)
            img.thumbnail((400,400))
            tkimage = ImageTk.PhotoImage(img)
            label.configure(image = tkimage)
            label.image = tkimage
            
            img1 = Image.fromarray(binary)
            img1.thumbnail((400,400))
            tkimage1 = ImageTk.PhotoImage(img1) 
            label1.configure(image = tkimage1)
            label1.image = tkimage1
            
            root.after(10,callback)
            
        else:
            onClossing()
            
########################### Ip Cam ###########################
            
url='http://172.29.61.28:8080/shot.jpg'
# url='http://192.168.0.17:8080/shot.jpg'

cap = cv2.VideoCapture(url)

if cap.isOpened():
    print("Ip Cam initializatized")
else:
    sys.exit("Ip Cam disconnected")

cap.open(url)    
ret, frame = cap.read()

####################### Desired position in pixels ##############

cxd = int(frame.shape[1]/2)
cyd = int(frame.shape[0]/2)

hxd = 0

########################### Serial communication ###########

port = 'COM4' 
arduino = serialArduino(port)
arduino.readSerialStart()

############################## HMI design #################

root = Tk()
root.protocol("WM_DELETE_WINDOW",onClossing)
root.title("Vision Artificial") # titulo de la ventana

label=Label(root)
label.grid(row=0,padx=20,pady=20)

label1=Label(root)
label1.grid(row= 0,column=1,padx=20,pady=20)

umbralValue = IntVar()
slider = Scale(root,label = 'Threshold value', from_=0, to=255, orient=HORIZONTAL,command=thresholdValue,length=400)   #Creamos un dial para recoger datos numericos
slider.grid(row = 1)

btnVar = StringVar(root, 'Pause')
btn = Checkbutton(root, text=btnVar.get(), width=12, variable=btnVar,
                  offvalue='Pause', onvalue='Start', indicator=False,
                  command=toggle)
btn.grid(row = 1,column = 1)

root.after(10,callback) #Es un método definido para todos los widgets tkinter.
root.mainloop()
