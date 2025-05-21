import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import matplotlib.pyplot as plt

class AUKFDetector:
    def __init__(self):
        # 初始化状态变量 [ID, 数据长度, 时间间隔]
        self.dim_x = 3
        self.dim_z = 3
        
        # 初始化UKF参数
        points = MerweScaledSigmaPoints(n=self.dim_x, 
                                      alpha=0.1, 
                                      beta=2.0, 
                                      kappa=0.0)
        
        self.ukf = UnscentedKalmanFilter(dim_x=self.dim_x,
                                       dim_z=self.dim_z,
                                       dt=1.0,
                                       hx=self.hx,
                                       fx=self.fx,
                                       points=points)
        
        # 初始化状态和协方差矩阵
        self.ukf.x = np.array([0, 0, 0])  # [ID, 数据长度, 时间间隔]
        self.ukf.P *= 100  # 初始不确定性较大
        
        # 过程噪声和观测噪声
        self.ukf.Q = np.eye(self.dim_x) * 0.1
        self.ukf.R = np.eye(self.dim_z) * 0.1
        
        # 存储历史数据用于可视化
        self.history = {'time': [], 'id': [], 'data_len': [], 'interval': []}
        self.residuals = []
        
    def fx(self, x, dt):
        """状态转移函数"""
        # 假设ID和数据长度保持不变，时间间隔增加
        return np.array([x[0], x[1], x[2] + dt])
    
    def hx(self, x):
        """观测函数"""
        return np.array([x[0], x[1], x[2]])
    
    def update(self, can_msg, timestamp):
        """处理新的CAN消息"""
        # 提取特征
        z = np.array([can_msg.arbitration_id,
                     len(can_msg.data),
                     timestamp - self.history['time'][-1] if self.history['time'] else 0])
        
        # 预测和更新
        self.ukf.predict()
        self.ukf.update(z)
        
        # 计算残差用于异常检测
        residual = np.linalg.norm(z - self.hx(self.ukf.x))
        self.residuals.append(residual)
        
        # 存储历史数据
        self.history['time'].append(timestamp)
        self.history['id'].append(can_msg.arbitration_id)
        self.history['data_len'].append(len(can_msg.data))
        self.history['interval'].append(z[2])
        
        # 自适应调整噪声
        self.adapt_noise(residual)
        
        return residual
    
    def adapt_noise(self, residual):
        """自适应调整过程噪声"""
        if residual > 1.0:  # 检测到异常
            self.ukf.Q *= 1.1  # 增加过程噪声
        else:
            self.ukf.Q *= 0.9  # 减少过程噪声
        self.ukf.Q = np.clip(self.ukf.Q, 0.01, 10)  # 限制噪声范围
    
    def plot_results(self):
        """绘制结果"""
        plt.figure(figsize=(12, 8))
        
        # 绘制ID变化
        plt.subplot(3, 1, 1)
        plt.plot(self.history['time'], self.history['id'], 'b-', label='Actual ID')
        plt.plot(self.history['time'], [self.ukf.x[0]]*len(self.history['time']), 'r--', label='Estimated ID')
        plt.legend()
        plt.title('CAN ID Tracking')
        
        # 绘制数据长度变化
        plt.subplot(3, 1, 2)
        plt.plot(self.history['time'], self.history['data_len'], 'b-', label='Actual Data Length')
        plt.plot(self.history['time'], [self.ukf.x[1]]*len(self.history['time']), 'r--', label='Estimated Data Length')
        plt.legend()
        plt.title('Data Length Tracking')
        
        # 绘制时间间隔变化
        plt.subplot(3, 1, 3)
        plt.plot(self.history['time'], self.history['interval'], 'b-', label='Actual Interval')
        plt.plot(self.history['time'], [self.ukf.x[2]]*len(self.history['time']), 'r--', label='Estimated Interval')
        plt.legend()
        plt.title('Time Interval Tracking')
        
        plt.tight_layout()
        plt.savefig('sim/aukf_tracking1.png')

if __name__ == "__main__":
    # 示例用法
    detector = AUKFDetector()
    
    # 模拟一些CAN消息
    class MockCANMessage:
        def __init__(self, id, data):
            self.arbitration_id = id
            self.data = data
    
    import time
    import random
    
    for i in range(100):
        msg = MockCANMessage(id=random.choice([0x123, 0x456]),
                           data=[random.randint(0, 255) for _ in range(random.randint(1, 8))])
        detector.update(msg, time.time())
        time.sleep(random.uniform(0.1, 1.0))
    
    detector.plot_results()
