import socket
import RPi.GPIO as gpio
import time

LEFT = 16
RIGHT = 18
MOTION = 23
SERVO = 19

gpio.setmode(gpio.BCM)
gpio.setup(LEFT, gpio.OUT)
gpio.setup(RIGHT, gpio.OUT)
gpio.setup(SERVO, gpio.OUT)
gpio.setup(MOTION, gpio.IN)
l = gpio.PWM(LEFT, 50)
r = gpio.PWM(RIGHT, 50)
servo = gpio.PWM(SERVO, 50)

HOST = ""
PORT = 3000

r.start(0)
l.start(0)
servo.start(0)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')
s.bind((HOST, PORT))
print ('Socket bind complete')
s.listen(1)
print ('Socket now listening')

def send(sock, res):
    sock.sendall(res.encode("utf-8"))
    print("Response: " + res)

def receive(sock):
    data = sock.recv(1024).decode("utf8").strip()
    print("Received: " + data)
    return sock

def play():
    servo.ChangeDutyCycle(12.5)
    #r.ChangeDutyCycle(0)
    #l.ChangeDutyCycle(0)

def find():
    #l.ChangeDutyCycle(10)
    #r.ChangeDutyCycle(5)
    servo.ChangeDutyCycle(0)

def feed():
    #r.ChangeDutyCycle(10)
    #l.ChangeDutyCycle(5)
    servo.ChangeDutyCycle(0)

def write_on_textView(data):

    if data == "clicked feed":
        data = "start feed"
        feed()

    elif data == "clicked play":
        data = "start play"
        play()

    elif data == "clicked find":
        data = "start find"
        find()

    else :
        data = "There is no comand like" + data


    return data

try:
    while True:
    
        conn, addr = s.accept()
        print("Connected by ", addr)

        data = conn.recv(1024)
        data = data.decode("utf8").strip()
        if not data: break
        print("Received: " + data)

        res = write_on_textView(data)
        print("Response: " + res)

        conn.sendall(res.encode("utf-8"))
        conn.close()

except KeyboardInterrupt :
    l.stop()
    servo.stop()
    r.stop()
    gpio.cleanup()
    s.close()
s.close()
