#!/usr/bin/python3

def is_port_in_use(port: int) -> bool:
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0
    
for port in range(2000, 3000):
    if not is_port_in_use(port):
        print(port)
        exit()