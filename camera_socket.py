import socket
import time
import struct
import picamera
import io

HOST = ""
PORT = 3000

# s : socket server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')
s.bind((HOST, PORT))
print ('Socket bind complete')
s.listen(1)
print ('Socket now listening')



def write_on_textView(data):
    if data == "clicked feed":
        data = "start feed"

    elif data == "clicked play":
        data = "start play"

    elif data == "clicked find":
        data = "start find"

    else:
        data = "There is no command like" + input_string
        
    return data
try:
    while True:
        conn, addr = s.accept()
        print("Connected by ", addr)

        data = conn.recv(1024)
        data = data.decode("utf8").strip()
        if not data: break
        print(data)

        print("Received: " + data)

        socket_client = conn.makefile('wb')
        print(socket_client)
        with picamera.PiCamera() as camera:
            camera.resolution=(320, 240)
            time.sleep(1)
            print ("Camera connected")
            stream=io.BytesIO()
            print ("Start streaming")
            for frameStream in camera.capture_continuous(stream, 'jpeg', True):
                socket_client.write(struct.pack(">I", frameStream.tell()))
                print(frameStream.tell())
                frameStream.seek(0)
                socket_client.write(frameStream.read())
                socket_client.flush()
                frameStream.seek(0)
                frameStream.truncate()

        print("Response: " + res)
        conn.sendall(res.encode("utf-8"))
        conn.close()

    s.close()
except KeyboardInterrupt:
    s.close()
