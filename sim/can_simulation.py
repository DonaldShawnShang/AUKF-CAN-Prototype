import can
import threading
import time
import random
from aukf_detection import AUKFDetector

def receiver_node():
    # 初始化AUKF检测器
    detector = AUKFDetector()
    with can.Bus(interface='socketcan', channel='vcan0') as bus:
        print("Receiver node started")
        listener = can.BufferedReader()
        notifier = can.Notifier(bus, [listener])
        
        try:
            while True:
                msg = listener.get_message(timeout=1.0)
                if msg:
                    # 发送响应消息
                    response = can.Message(
                        arbitration_id=0x456,
                        data=[0xAA, 0xBB],
                        is_extended_id=False
                    )
                    bus.send(response)
                    # 使用candump格式显示消息
                    timestamp = time.time()
                    print(f"({timestamp:.6f}) vcan0 {msg.arbitration_id:03X}#{msg.data.hex()}")
                    print(f"({timestamp:.6f}) vcan0 {response.arbitration_id:03X}#{response.data.hex()}")
                    
                    # 进行异常检测
                    residual = detector.update(msg, timestamp)
                    if residual > 2.0:  # 设置异常阈值
                        print(f"WARNING: Potential intrusion detected! Residual: {residual:.2f}")
                    
                    # 添加自动暂停
                    time.sleep(2.0)  # 暂停2秒
        finally:
            notifier.stop()

def sender_node():
    time.sleep(1)  # 等待接收方启动
    with can.Bus(interface='socketcan', channel='vcan0') as bus:
        print("Sender node started")
        listener = can.BufferedReader()
        notifier = can.Notifier(bus, [listener])
        
        try:
            while True:  # 持续发送消息
                # 发送消息，随机插入异常
                if random.random() < 0.1:  # 10%概率发送异常消息
                    msg = can.Message(
                        arbitration_id=random.choice([0x999, 0x888]),
                        data=[random.randint(0, 255) for _ in range(8)],
                        is_extended_id=False
                    )
                else:
                    msg = can.Message(
                        arbitration_id=0x123,
                        data=[0x01, 0x02, 0x03, 0x04],
                        is_extended_id=False
                    )
                bus.send(msg)
                # 使用candump格式显示发送的消息
                timestamp = time.time()
                print(f"({timestamp:.6f}) vcan0 {msg.arbitration_id:03X}#{msg.data.hex()}")
                
                # 等待响应
                response = listener.get_message(timeout=2.0)
                if response:
                    timestamp = time.time()
                    print(f"({timestamp:.6f}) vcan0 {response.arbitration_id:03X}#{response.data.hex()}")
                else:
                    print("No response received")
                
                time.sleep(10.0)  # 每次发送后等待1秒
        finally:
            notifier.stop()

if __name__ == "__main__":
    # 启动接收方线程
    receiver_thread = threading.Thread(target=receiver_node)
    receiver_thread.start()
    
    # 启动发送方
    sender_node()
    
    # 等待接收方线程结束
    receiver_thread.join()
