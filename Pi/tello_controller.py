import socket
import threading
import time
import tellopy

class TelloController:
    def __init__(self, ip_address='0.0.0.0', port=9000):
        self.ip_address = ip_address
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip_address, port))
        self.tello = tellopy.Tello()
        
    def start(self):
        self._listen_for_messages()

    def _listen_for_messages(self):
        while True:
            data, address = self.sock.recvfrom(1024)
            message = data.decode()
            print(f"Received message: {message}")

            if message == "takeoff":
                self.tello.takeoff()
                time.sleep(5)
                self.tello.up(200)
                print("Tello took off and is going up...")
                self._take_picture()
                self.tello.down(self.tello.get_height()) # Go down to minimum height

    def _take_picture(self):
        # Take picture and get the data as bytes
        self.tello.take_picture()
        picture_data = self.tello.get_picture()

        # Send picture data to the client
        self.sock.sendto(picture_data, (self.ip_address, self.port+1)) # Port+1 to send to client
