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

        self.waiting_time = 0.2
        self.command_waiting_time = 0.5
        self.serial.timeout = 0.5
        self.serial.writeTimeout = 0.5

        # 每1/30秒產生的編碼器脈衝數
        self.ticks = 15
        # 輪子直徑
        self.wheel_diameter = 0.1260
        # 左右輪間距
        self.wheel_track = 0.3500
        # 輪子轉一圈產生的脈衝數
        self.encoder_resolution = 1200
        # PWM調節頻率
        self.PID_RATE = 30
        self.ticks_per_meter = self.encoder_resolution / (self.wheel_diameter * math.pi)
        self.speed = self.ticks * self.PID_RATE / self.ticks_per_meter

        self.input_queue = Queue()
        self.output_queue = Queue()
        self.semaphore = threading.Semaphore(2)
        self.stop_event = threading.Event()
        self.t1 = threading.Thread(target=self.execute, args=())
        self.t1.start()

    # 旋轉
    def spin(self, direction=True):
        print("Spinning.")
        self.stop_event.clear()
        while not self.stop_event.wait(0):
            if direction:
                s = "z %s -%s\r" % (self.ticks, self.ticks)
            else:
                s = "z -%s %s\r" % (self.ticks, self.ticks)
            self.input_queue.put(s)
            time.sleep(self.command_waiting_time)
        self.pause()

    # 前進
    def forward(self):
        print("Forwarding.")
        self.stop_event.clear()
        while not self.stop_event.wait(0):
            s = "z %s %s\r" % (self.ticks, self.ticks)
            self.input_queue.put(s)
            time.sleep(self.command_waiting_time)
        self.pause()

    # 後退
    def backward(self):
        print("Backwarding.")
        self.stop_event.clear()
        while not self.stop_event.wait(0):
            s = "z -%s -%s\r" % (self.ticks, self.ticks)
            self.input_queue.put(s)
            time.sleep(self.command_waiting_time)
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

    def second(self, distance):
        if distance < 100:
            return distance / self.speed / 100 - 0.5
        else:
            return distance / self.speed / 100 - 1

    def execute(self):
        while True:
            cammand = self.input_queue.get()
            self.semaphore.acquire()
            try:
                self.serial.write(cammand.encode())
                output = self.serial.read_until("\r").decode()
                if output != "":
                    self.output_queue.put(output)
            except:
                print("execute exception")
                self.output_queue.put("20 20 20 20")
            self.semaphore.release()

    # 超音波測距
    def ping(self):
        data = [0] * 4
        self.input_queue.put("p\r")
        data = list(map(int, self.output_queue.get().split()))
        print(data)
        return data

    def clear(self):
        while not self.input_queue.empty():
            self.input_queue.get()
        self.pause()
def main():
    # 電腦
    s = D1("COM3", 115200)

    # 樹莓派
    # s = D1("/dev/ttyUSB0", 115200)

    try:
        while True:
            time.sleep(s.waiting_time)
            data = s.ping()

            obstacle = False
            for i in range(3):
                if 0 < data[i] < 35:
                    obstacle = True
                    break

            if obstacle:
                # 後退
                distance = data[3] if 0 < data[3] < 50 else 50
                t2 = threading.Thread(target=s.backward, args=())
                t2.start()
                t2.join(s.second(distance))
                s.stop_event.set()

                s.clear()
                time.sleep(s.waiting_time)

                # 轉向
                t3 = threading.Thread(target=s.spin, args=(True,))
                t3.start()
                t3.join(1.5)
                s.stop_event.set()

            else:
                # 前進
                distance = min(data[i] for i in range(3) if data[i] > 0)
                t4 = threading.Thread(target=s.forward, args=())
                t4.start()
                t4.join(s.second(distance))
                s.stop_event.set()

            s.clear()
            print("next")
    except KeyboardInterrupt as e:
        print(e)
        exit(0)

if __name__ == '__main__':
    main()
