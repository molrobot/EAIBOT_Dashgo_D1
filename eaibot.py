import serial
import threading
import time
import math
from queue import Queue



class D1(threading.Thread):
    def __init__(self, com_port, baudrate=115200):
        threading.Thread.__init__(self)
        try:
            self.serial = serial.Serial(com_port, baudrate)
            time.sleep(1)
            if self.serial.isOpen():
                print("Successfully")
        except serial.SerialException:
            print("Cannot connect device " + com_port)
            exit(1)

        self.waiting_time = 0.4
        self.command_waiting_time = 0.5
        self.serial.timeout = 0.5
        self.serial.writeTimeout = 0.5

        self.input_queue = Queue()
        self.output_queue = Queue()
        self.semaphore = threading.Semaphore(2)
        self.stop_event = threading.Event()
        t1 = threading.Thread(target=self.execute, args=())
        t1.start()
        
    # 旋轉
    def spin(self, speed=5, direction=True):
        print("Spinning.")
        self.stop_event.clear()
        while not self.stop_event.wait(0):
            if direction:
                s = "z %s -%s\r" % (speed, speed)
            else:
                s = "z -%s %s\r" % (speed, speed)
            self.input_queue.put(s)
            time.sleep(self.command_waiting_time)
        self.pause()
    
    # 前進
    def forward(self, speed=5):
        print("Forwarding.")
        self.stop_event.clear()
        while not self.stop_event.wait(0):
            s = "z %s %s\r" % (speed, speed)
            self.input_queue.put(s)
            time.sleep(self.command_waiting_time)
        self.pause()

    # 後退
    def backward(self, speed=5):
        print("Backwarding.")
        self.stop_event.clear()
        while not self.stop_event.wait(self.command_waiting_time):
            s = "z -%s -%s\r" % (speed, speed)
            self.input_queue.put(s)
        self.pause()

    # 左轉
    def left(self, speed):
        print("Turning left.")
        
        self.pause()

    # 右轉
    def right(self, speed):
        print("Turning right.")
        
        self.pause()
        
    # 暫停
    def pause(self):
        self.stop_event.set()
        s = "z 0 0\r"
        try:
            self.serial.write(s.encode())
        except:
            pass

    def speed(self):
        pass

    def execute(self):
        while True:
            cammand = self.input_queue.get()
            self.semaphore.acquire()
            try:
                self.serial.flush()
                self.serial.write(cammand.encode())
                time.sleep(self.command_waiting_time)
                while self.serial.in_waiting:
                    self.output_queue.put(self.serial.readline().decode())
            except:
                pass
            self.semaphore.release()

    # 超音波測距
    def ping(self):
        data = [0] * 4
        self.input_queue.put("p\r")
        data = list(map(int, self.output_queue.get().split()))
        print(data)
        return data

def main():
    # 電腦
    s = D1("COM3", 115200)

    # 樹莓派
    # s = D1("/dev/ttyUSB0", 115200)

    while True:
        time.sleep(s.waiting_time)
        data = s.ping()

        obstacle = False
        for i in range(3):
            if 0 < data[i] < 20:
                obstacle = True
                break

        # 後退
        # 轉向
        if obstacle:
            t2 = threading.Thread(target=s.spin, args=(10,True))
            t2.start()
            time.sleep(3)
            s.stop_event.set()
        else:
            # 前進
            t2 = threading.Thread(target=s.forward, args=(10,))
            t2.start()
            time.sleep(2)
            s.stop_event.set()
        while not s.input_queue.empty():
            s.input_queue.get()
        print("next")

if __name__ == '__main__':
    main()
