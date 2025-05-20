import json
from IMU.imu_receiver import read_imu_data

def get_imu_data():
    imu_data = read_imu_data()
    return json.dumps(imu_data)  # 以 JSON 形式返回数据