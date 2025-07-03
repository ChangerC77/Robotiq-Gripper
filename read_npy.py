import numpy as np

# 读取.npy文件
data = np.load('data/0006.npy')

# 查看数据内容
print(data)          # 打印数组内容
print(data.shape)    # 打印数组形状
print(data.dtype)    # 打印数据类型