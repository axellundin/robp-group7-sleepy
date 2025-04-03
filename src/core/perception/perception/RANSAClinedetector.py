import numpy as np
import random
import matplotlib.pyplot as plt

import random
import numpy as np

class RANSACLineDetector:
    def __init__(self, ransac_n=2, max_dst=0.04, max_trials=1000, stop_inliers_ratio=1.0, max_lines=2):
        self.ransac_n = ransac_n  # RANSAC采样的点数，这里是2，因为每条直线需要两个点
        self.max_dst = max_dst    # 最大距离阈值，判断点是否是内点
        self.max_trials = max_trials  # 最大的RANSAC迭代次数
        self.stop_inliers_ratio = stop_inliers_ratio  # 内点比例达到该值时停止RANSAC
        self.max_lines = max_lines  # 假设最多2条直线

    class line_model:
        def __init__(self):
            self.parameters = None  # 用于存储直线的参数，通常是一个法向量和中心点

        def calc_inliers(self, points, dst_threshold):
            """
            计算哪些点是内点，返回布尔值的列表
            """
            p1, p2 = self.parameters  # 直线的两个点
            # 计算每个点到直线的距离
            line_vec = p2 - p1
            point_vec = points - p1
            cross_product = np.cross(line_vec, point_vec)
            dst = np.abs(cross_product) / np.linalg.norm(line_vec)  # 点到直线的距离
            return dst < dst_threshold  # 返回一个布尔值的数组，标记哪些点是内点

        def estimate_parameters(self, pts):
            if pts.shape[0] == 2:
                p1, p2 = pts
                self.parameters = (p1, p2)
                return self.parameters
            else:
                raise ValueError("At least two points are required to define a line.")

    def ransac_linefit(self, points):
        pts = points.copy()  # 拷贝点数据
        num = pts.shape[0]  # 点的数量
        best_inliers_ratio = 0
        best_line_params = None
        best_inliers = None
        best_outliers = None
        
        for _ in range(self.max_trials):
            # 随机选择两个点来拟合直线
            sample_index = random.sample(range(num), self.ransac_n)
            sample_points = pts[sample_index, :]
            
            line = self.line_model()
            line_params = line.estimate_parameters(sample_points)

            # 计算内点
            inliers = line.calc_inliers(points, self.max_dst)
            inliers_ratio = np.sum(inliers) / num

            if inliers_ratio > best_inliers_ratio:
                best_inliers_ratio = inliers_ratio
                best_line_params = line_params
                best_inliers = pts[inliers]
                best_outliers = pts[~inliers]

            if best_inliers_ratio > self.stop_inliers_ratio:
                break

        return best_line_params, best_inliers, best_outliers

    def ransac_line_detection(self, points):
        """
        检测多个直线，返回每条直线的参数、内点和外点
        """
        line_set = []
        line_inliers_set = []
        data_remains = np.copy(points)

        line_count = 0  # 跟踪找到的直线数量

        while len(data_remains) > 3 and line_count < self.max_lines:
            # 用RANSAC拟合一条直线
            best_line_params, inliers, outliers = self.ransac_linefit(data_remains)
            if best_line_params is not None:
                line_set.append(best_line_params)
                line_inliers_set.append(inliers)
                data_remains = outliers  # 更新剩余的点集
                line_count += 1  # 找到一条直线，计数加1
            else:
                break  # 没有找到新的直线

        return line_set, line_inliers_set, data_remains



# 测试代码
if __name__ == '__main__':
    # 创建一些测试点
    points = np.array([
        [1, 1],
        [2, 2],
        [3, 3],
        [4, 4],
        [10, 10],
        [11, 11],
        [12, 12],
    ])

    # 初始化RANSAC直线检测器
    line_detector = RANSACLineDetector(ransac_n=2, max_dst=0.1, max_trials=100, stop_inliers_ratio=0.7)

    # 检测直线
    line_set, line_inliers_set, remains = line_detector.ransac_line_detection(points)

    # 输出结果
    print("Detected lines:")
    for i, line in enumerate(line_set):
        print(f"Line {i + 1}:")
        print(f"  Points: {line[0]}, {line[1]}")
        print(f"  Inliers: {line_inliers_set[i]}")
    
    # 可视化检测结果
    plt.scatter(points[:, 0], points[:, 1], color='blue', label='Points')
    for line in line_set:
        p1, p2 = line
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], label='Fitted Line')
    plt.legend()
    plt.show()
