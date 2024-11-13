import socket
localhost='192.168.31.67'
# TCP发送
def tcp_send(host, port, message):
    # 创建TCP套接字
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
        # 连接到指定的主机和端口
        tcp_socket.connect((host, port))
        # 发送消息
        tcp_socket.sendall(message.encode())
        print("TCP message sent:", message)

# UDP接收
def udp_receive(host, port):
    # 创建UDP套接字
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        # 绑定到指定的主机和端口
        udp_socket.bind((host, port))
        print("UDP server is listening...")
        while True:
            # 接收数据
            data, addr = udp_socket.recvfrom(1024)
            print("UDP message received:", data.decode(), "from", addr)

if __name__ == "__main__":
    # 主机和端口
    tcp_host = localhost
    tcp_port = 800
    udp_host = '0.0.0.0'
    udp_port = 1234

    # 发送TCP消息
    tcp_send(tcp_host, tcp_port, "Hello TCP!")

    # 开启UDP接收
    udp_receive(udp_host, udp_port)
