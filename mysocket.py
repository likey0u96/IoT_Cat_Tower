import socket

HOST = ""
PORT = 3000

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

	else :
		data = "There is no comand like" + input_string

	return data

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

s.close()
