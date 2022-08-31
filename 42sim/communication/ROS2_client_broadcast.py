import socket
import threading
import time

IPADDR = "127.0.0.1"
PORT = 49152

sock = socket.socket(socket.AF_INET)
sock.connect((IPADDR, PORT))

# データ受信関数
def recv_data(sock):
    while True:
        try:
            #data = sock.recv(32768)
            data = sock.recv(1024)

            if data == b"":
                break
            print(data.decode("utf-8"))
        except ConnectionResetError:
            break

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

# データ受信をサブスレッドで実行
thread = threading.Thread(target=recv_data, args=(sock,))
thread.start()

# データ入力ループ

i = 0
#while True:
while True:
    data = input("> ")
    
    '''
    time.sleep(0.01)
    data = "     SC[0].AC.Whl[0].Tcmd = 0"
    i = i + 1

    #print(data)

    if i >= 100:
        data = "     SC[0].AC.Whl[0].Tcmd = 1.0"
    
    if i == 5000:
        break
    
    '''


    if data == "exit":
        break
    else:
        try:
            sock.send(data.encode("utf-8"))
        except ConnectionResetError:
            break

sock.shutdown(socket.SHUT_RDWR)
sock.close()