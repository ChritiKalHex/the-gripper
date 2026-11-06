# listener.py
import socket
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 8080))
s.listen()
while True:
    conn, addr = s.accept()
    print("connected:", addr)
    with conn, conn.makefile() as f:
        for line in f:
            print("got:", line.strip())