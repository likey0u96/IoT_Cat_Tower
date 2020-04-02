from threading import Thread
from time import sleep
import socket
import RPi.GPIO as gpio
import time
import paho.mqtt.client as mqtt
import json
import math

#servo
LEFT = 16
RIGHT = 18
#Ultrasonic sensors
TRIG = 13
ECHO = 26
#servo
gpio.setmode(gpio.BCM)
gpio.setup(LEFT, gpio.OUT)
gpio.setup(RIGHT, gpio.OUT)
#Ultrasonic sensors
gpio.setup(TRIG, gpio.OUT)
gpio.setup(ECHO, gpio.IN)
gpio.output(TRIG, gpio.LOW)
#servo
l = gpio.PWM(LEFT, 50)
r = gpio.PWM(RIGHT, 50)
#servo = gpio.PWM(SERVO, 50)
r.start(0)
l.start(0)
#servo.start(0)

HOST = ""
PORT = 3000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

print ('Socket1 created')
s.bind((HOST, PORT))
print ('Socket1 bind complete')
s.listen(1)
print ('Socket1 now listening')

data = ""

mqttflag = 0
dis = 0.0
dis_before = 0.0
cdnt = 0

towerxloc=0
toweryloc=0
catxloc=0
catyloc=0

micro = 0

#Ultrasonic sensor distance
def check_Ultrasonic() :
    gpio.output(TRIG, gpio.HIGH)
    time.sleep(0.00001)
    gpio.output(TRIG, gpio.LOW)
    stop = 0
    start = 0
    while gpio.input(ECHO)==gpio.LOW :
        start = time.time()
    while gpio.input(ECHO)==gpio.HIGH :
        stop = time.time()
    duration = stop - start
    ultradis = (duration*340*100)/2

    return ultradis


#for mqtt connect
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    client.subscribe("towerInfo")

#for mqtt subscribe
def on_message(client, userdata, msg):
    global dis
    global dis_before
    global towerxloc
    global toweryloc
    global catxloc
    global catyloc
    
    if mqttflag == 1:
        dis_before = dis
        recvData=str(msg.payload)
        jsonData=json.loads(recvData)
        
        dis = float(str(jsonData["distance"]))
        towerxloc = float(str(jsonData["towerxloc"]))
        toweryloc = float(str(jsonData["toweryloc"]))
        catxloc = float(str(jsonData["catxloc"]))
        catyloc = float(str(jsonData["catyloc"]))
        
#mqtt main
def mqtt_sub():
    client = mqtt.Client()
    print("client created")
    client.on_connect = on_connect
    print("connected")
    client.on_message = on_message
    client.connect("192.168.1.3",1883,60) #cat ip
    client.loop_forever()


def check():
    if data == "clicked feed" :
        feed()
    elif data == "clicked play":
        play()
    elif data == "clicked find":
        find()
        
def play():
    global l
    global r
    #global servo
    #servo.ChangeDutyCycle(12.5)

def find():
    global l
    global r
    #global servo

    global dis
    global dis_before
    global cnt

    global micro #Ultrasonic sensors
    
    global catxloc
    global catyloc
    global towerxloc
    global toweryloc
    
    out = 0 #out => (min,max) = (0,1)
    turn = 0 #trun => (min,max) = (0,2)

    while True : 
        micro = check_Ultrasonic()
        if micro < 5 :
            #back
            l.ChangeDutyCycle(5)
            r.ChangeDutyCycle(10)
            sleep(1.5)
        if out == 0 :
            if catxloc == towerxloc and catyloc == toweryloc :
                out = 0
                turn = 0
                return
            else :
                if toweryloc != 1 :
                    #go straight
                    l.ChangeDutyCycle(10)
                    l.ChangeDutyCycle(5)
                    sleep(2)
                else:
                    out = 1
                    
        else : #out room
            if catxloc == towerxloc and catyloc == toweryloc :
                return
            else :
                if toweryloc == 1 :
                    if turn == 0 :
                        if towerxloc == catxloc :
                            #go straight
                            l.ChangeDutyCycle(10)
                            r.ChangeDutyCycle(5)
                            sleep(2)
                        else :
                            # turn 90
                            l.ChangeDutyCycle(10)
                            r.ChangeDutyCycle(15)
                            sleep(0.8)
                            turn = 1
                    elif turn == 1 : #toweryloc = 1 and turn = 1
                        if towerxloc != catxloc :
                            if dis < dis_before :
                                #go straight
                                l.ChangeDutyCycle(10)
                                r.ChangeDutyCycle(5)
                                sleep(2)
                            else :
                                #turn 180 + straight
                                l.ChangeDutyCycle(10)
                                r.ChangeDutyCycle(10)
                                sleep(1.5)
                                l.ChangeDutyCycle(10)
                                r.ChangeDutyCycle(5)
                                sleep(2)
                        else : #towerxloc = catxloc
                            #turn 90
                            l.ChangeDutyCycle(10)
                            r.ChangeDutyCycle(10)
                            sleep(0.8)
                            turn = 2
            if turn == 2 :
                if toweryloc != catyloc :
                    if dis < dis_before :
                        #go straight
                        l.ChangeDutyCycle(10)
                        r.ChangeDutyCycle(5)
                        sleep(1)
                    else :
                        #trun 180 and go straight
                        l.ChangeDutyCycle(10)
                        r.ChangeDutyCycle(10)
                        sleep(1.5)
                        l.ChangeDutyCycle(10)
                        r.ChangeDutyCycle(5)
                        sleep(2)

    
def feed():
    global l
    global r
    #global servo
    print("feed feed!")
    #r.ChangeDutyCycle(0)
    #l.ChangeDutyCycle(0)
    #servo.ChangeDutyCycle(0)

def sock():
    global data
    while True:
        conn, addr = s.accept()
        print("Connected by ", addr)
        res = conn.recv(1024)
        res = res.decode("utf8").strip()
        if not res: break
        print("Received: " + res)
        conn.sendall(res.encode("utf-8"))
        data = res
        conn.close()

    return

def ilovecat():
    global data
    global MOTION
    global mqttflag
    try:
        while True:
            #if gpio.input(MOTION) == gpio.LOW:
                #print("motion detected")
                #play()
            if data == "clicked feed" :
                feed()
                mqttflag = 0
            elif data == "clicked play":
                play()
                mqttflag = 0
            elif data == "clicked find":
                find()
                mqttflag = 1
            else :
                print("data",data)
            sleep(2)
            
    except KeyboardInterrupt :
        l.stop()
        r.stop()
        gpio.cleanup()

    return

if __name__ == "__main__":
    tmp = None 
    th1 = Thread(target=sock, args=())
    th2 = Thread(target=ilovecat, args=())
    th3 = Thread(target=mqtt_sub, args=())
    
    th1.start()
    th2.start()
    th3.start()
    th1.join()
    th2.join()
    th3.join()

