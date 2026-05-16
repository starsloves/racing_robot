import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import sys

class ImuDriftTester(Node):
    def __init__(self):
        super().__init__('imu_drift_tester')
        
        # 订阅你的 BNO055 话题
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
            
        # 设定收集的样本数量 (10Hz 下 150 个样本大约需要 15 秒)
        self.target_samples = 150
        self.current_samples = 0
        
        # 数据存储容器
        self.gyro = {'x': [], 'y': [], 'z': []}
        self.accel = {'x': [], 'y': [], 'z': []}
        
        self.get_logger().info('======================================')
        self.get_logger().info('🚀 零偏测试节点已启动！')
        self.get_logger().info('⚠️  警告：请立刻将小车放置在平稳地面上，【绝对不要触碰或晃动】！')
        self.get_logger().info(f'正在收集 {self.target_samples} 个静止样本进行统计...')
        self.get_logger().info('======================================')

    def imu_callback(self, msg):
        if self.current_samples < self.target_samples:
            # 记录角速度 (陀螺仪)
            self.gyro['x'].append(msg.angular_velocity.x)
            self.gyro['y'].append(msg.angular_velocity.y)
            self.gyro['z'].append(msg.angular_velocity.z)
            
            # 记录线加速度
            self.accel['x'].append(msg.linear_acceleration.x)
            self.accel['y'].append(msg.linear_acceleration.y)
            self.accel['z'].append(msg.linear_acceleration.z)
            
            self.current_samples += 1
            
            # 每收集 30 个样本打印一次进度
            if self.current_samples % 30 == 0:
                self.get_logger().info(f'进度: {self.current_samples} / {self.target_samples} 样本...')
                
            # 收集完毕，开始计算
            if self.current_samples == self.target_samples:
                self.calculate_and_report()
                self.get_logger().info('✅ 测试完成，正在退出...')
                # 抛出异常以优雅地结束节点
                raise SystemExit

    def compute_stats(self, data_list):
        # 计算均值 (Mean) - 这就是零偏
        mean = sum(data_list) / len(data_list)
        # 计算方差和标准差 (Std Dev) - 这就是噪音
        variance = sum((x - mean) ** 2 for x in data_list) / len(data_list)
        std_dev = math.sqrt(variance)
        return mean, std_dev

    def calculate_and_report(self):
        print("\n\n" + "="*50)
        print(" 📊 BNO055 零偏与底噪测试报告 ")
        print("="*50)
        
        # 计算陀螺仪统计
        gx_m, gx_s = self.compute_stats(self.gyro['x'])
        gy_m, gy_s = self.compute_stats(self.gyro['y'])
        gz_m, gz_s = self.compute_stats(self.gyro['z'])
        
        # 计算加速度计统计
        ax_m, ax_s = self.compute_stats(self.accel['x'])
        ay_m, ay_s = self.compute_stats(self.accel['y'])
        az_m, az_s = self.compute_stats(self.accel['z'])
        
        print("\n🌀 陀螺仪 (角速度 Angular Velocity) [单位: rad/s]")
        print(" -> 理想静止状态下，均值应无限接近 0。")
        print(f"  [X 轴 (Roll)]  均值(零偏): {gx_m: .6f} | 标准差(底噪): {gx_s: .6f}")
        print(f"  [Y 轴 (Pitch)] 均值(零偏): {gy_m: .6f} | 标准差(底噪): {gy_s: .6f}")
        print(f"  [Z 轴 (Yaw)]   均值(零偏): {gz_m: .6f} | 标准差(底噪): {gz_s: .6f}")
        
        print("\n🚀 加速度计 (线加速度 Linear Acceleration) [单位: m/s^2]")
        print(" -> BNO055内部已滤除重力，因此理想静止状态下三轴均值应无限接近 0。")
        print(f"  [X 轴 (前/后)] 均值(零偏): {ax_m: .6f} | 标准差(底噪): {ax_s: .6f}")
        print(f"  [Y 轴 (左/右)] 均值(零偏): {ay_m: .6f} | 标准差(底噪): {ay_s: .6f}")
        print(f"  [Z 轴 (上/下)] 均值(零偏): {az_m: .6f} | 标准差(底噪): {az_s: .6f}")
        
        print("\n💡 结论建议：")
        print(" 1. Z轴陀螺仪均值如果在 0.00x 级别，说明航向极度稳定。")
        print(" 2. 如果你在 EKF 配置中需要填方差矩阵 (Covariance)，可以直接平方这里的【标准差】填进去！")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    tester = ImuDriftTester()
    try:
        rclpy.spin(tester)
    except SystemExit:
        pass # 正常退出
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()