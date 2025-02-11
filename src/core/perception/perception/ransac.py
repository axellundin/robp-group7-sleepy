import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import math
import random

class RANSACPlaneDetector:
    def __init__(self, ransac_n=3, max_dst=0.05, max_trials=1000, stop_inliers_ratio=1.0, out_layer_inliers_threshold=230, out_layer_remains_threshold=230):
        self.ransac_n = ransac_n
        self.max_dst = max_dst
        self.max_trials = max_trials
        self.stop_inliers_ratio = stop_inliers_ratio
        self.out_layer_inliers_threshold = out_layer_inliers_threshold
        self.out_layer_remains_threshold = out_layer_remains_threshold

    def SVD(self, points):
        # 二维，三维均适用
        # 二维直线，三维平面
        pts = points.copy()
        # 奇异值分解
        c = np.mean(pts, axis=0)
        A = pts - c # shift the points
        A = A.T #3*n
        u, s, vh = np.linalg.svd(A, full_matrices=False, compute_uv=True) # A=u*s*vh
        normal = u[:,-1]

        # 法向量归一化
        nlen = np.sqrt(np.dot(normal,normal))
        normal = normal / nlen
        # normal 是主方向的方向向量 与PCA最小特征值对应的特征向量是垂直关系
        # u 每一列是一个方向
        # s 是对应的特征值
        # c >>> 点的中心
        # normal >>> 拟合的方向向量
        return u,s,c,normal


    class plane_model(object):
        def __init__(self):
            self.parameters = None

        def calc_inliers(self,points,dst_threshold):
            c = self.parameters[0:3]#center？
            n = self.parameters[3:6]#normal？
            dst = abs(np.dot(points-c,n))
            ind = dst<dst_threshold
            return ind#是a list showing points are inliers or not 

        def estimate_parameters(self,pts):
            num = pts.shape[0]
            if num == 3:
                c = np.mean(pts,axis=0)
                l1 = pts[1]-pts[0]
                l2 = pts[2]-pts[0]
                n = np.cross(l1,l2)
                scale = [n[i]**2 for i in range(n.shape[0])]
                #print(scale)
                n = n/np.sqrt(np.sum(scale))
            else:
                _,_,c,n = self.SVD(pts)

            params = np.hstack((c.reshape(1,-1),n.reshape(1,-1)))[0,:]
            self.parameters = params
            return params

        def set_parameters(self,parameters):
            self.parameters = parameters


    def ransac_planefit(self, points, ransac_n, max_dst, max_trials=1000, stop_inliers_ratio=1, initial_inliers=None):
        # RANSAC 平面拟合
        pts = points.copy()#一个三维坐标
        num = pts.shape[0]#多少行 多少个点
        cc = np.mean(pts,axis=0)#中心点
        iter_max = max_trials
        best_inliers_ratio = 0 #最佳内点比例 迭代越多越好
        best_plane_params = None
        best_inliers = None
        best_remains = None
        for i in range(iter_max):
            sample_index = random.sample(range(num),ransac_n)#从范围num选ransac_n个
            sample_points = pts[sample_index,:]
            plane = self.plane_model()
            plane_params = plane.estimate_parameters(sample_points)
            #  计算内点 index是一个指示是不是内点的列表
            index = plane.calc_inliers(points,max_dst)
            inliers_ratio = pts[index].shape[0]/num

            if inliers_ratio > best_inliers_ratio:
                best_inliers_ratio = inliers_ratio
                best_plane_params = plane_params
                bset_inliers = pts[index]
                bset_remains = pts[index==False]

            if best_inliers_ratio > stop_inliers_ratio:
                # 检查是否达到最大的比例
                print("iter: %d\n" % i)
                print("best_inliers_ratio: %f\n" % best_inliers_ratio)
                break

        return best_plane_params,bset_inliers,bset_remains


    def ransac_plane_detection(self, points, ransac_n, max_dst, max_trials=1000, stop_inliers_ratio=1.0, initial_inliers=None, out_layer_inliers_threshold=230, out_layer_remains_threshold=230):
        """
        Detects a set of planes from the input points using RANSAC.
        
        :param points: A np matrix of points with only 3D positions.
        :param ransac_n: The number of points to sample for RANSAC.
        :param max_dst: The maximum distance for points to be considered as inliers.
        :param max_trials: The maximum number of RANSAC iterations.
        :param stop_inliers_ratio: The ratio of inliers to stop the RANSAC process.
        :param initial_inliers: Initial inliers (if any).
        :param out_layer_inliers_threshold: Threshold for the number of inliers to consider.
        :param out_layer_remains_threshold: Threshold for the number of remaining points.
        :return: Detected plane parameters and inliers.
        """
        inliers_num = out_layer_inliers_threshold + 1
        remains_num = out_layer_remains_threshold + 1

        plane_set = []
        plane_inliers_set = []
        plane_inliers_num_set = []

        data_remains = np.copy(points)

        i = 0

        while inliers_num>out_layer_inliers_threshold and remains_num>out_layer_remains_threshold:#如果内点太少就结束了 如果外点太少说明真该结束了
            # robustly fit line only using inlier data with RANSAC algorithm
            best_plane_params,pts_inliers,pts_outliers = self.ransac_planefit(data_remains, ransac_n, max_dst, max_trials=max_trials, stop_inliers_ratio=stop_inliers_ratio)

            inliers_num = pts_inliers.shape[0]
            remains_num = pts_outliers.shape[0]

            if inliers_num>out_layer_inliers_threshold:
                plane_set.append(best_plane_params)
                plane_inliers_set.append(pts_inliers)
                plane_inliers_num_set.append(inliers_num)
                i = i+1
                print('------------> %d <--------------' % i)
                print(best_plane_params)

            data_remains = pts_outliers

        # sorting #按照内点降序排序
        plane_set = [x for _, x in sorted(zip(plane_inliers_num_set,plane_set), key=lambda s: s[0], reverse=True)]
        plane_inliers_set = [x for _, x in sorted(zip(plane_inliers_num_set,plane_inliers_set), key=lambda s: s[0], reverse=True)]

        return plane_set, plane_inliers_set, data_remains
        

    def show_3dpoints(self, pointcluster,s=None,colors=None,quiver=None,q_length=10,tri_face_index=None):
        # pointcluster should be a list of numpy ndarray
        # This functions would show a list of pint cloud in different colors
        n = len(pointcluster)
        if colors is None:
            colors = ['r','g','b','c','m','y','k','tomato','gold']
            if n < 10:
                colors = np.array(colors[0:n])
            else: 
                colors = np.random.rand(n,3)
            
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        if s is None:
            s = np.ones(n)*2

        for i in range(n):
            ax.scatter(pointcluster[i][:,0],pointcluster[i][:,1],pointcluster[i][:,2],s=s[i],c=[colors[i]],alpha=0.6)

        if not (quiver is None):
            c1 = [random.random() for _ in range(len(quiver))]
            c2 = [random.random() for _ in range(len(quiver))]
            c3 = [random.random() for _ in range(len(quiver))]
            c = []
            for i in range(len(quiver)):
                c.append((c1[i],c2[i],c3[i]))
            cp = []
            for i in range(len(quiver)):
                cp.append(c[i])
                cp.append(c[i])
            c = c + cp
            ax.quiver(quiver[:,0],quiver[:,1],quiver[:,2],quiver[:,3],quiver[:,4],quiver[:,5],length=q_length,arrow_length_ratio=.2,pivot='tail',normalize=False,color=c)
        
        if not (tri_face_index is None):
            for i in range(len(tri_face_index)):
                for j in range(tri_face_index[i].shape[0]):
                    index = tri_face_index[i][j].tolist()
                    index = index + [index[0]]
                    ax.plot(*zip(*pointcluster[i][index]))

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.show()

        return 0