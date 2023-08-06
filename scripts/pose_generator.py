#!/usr/bin/python
#-*- coding:utf-8 -*-

import numpy as np
import random
from copy import deepcopy
from scipy.spatial.transform import Rotation as R

mu_rotation = 0
mu_translation = 0
sigma_rotation = 0.01
sigma_translation = 0.2

def quat2mat(pose):
    pose_updated = np.identity(4)
    pose_updated[:-1, :-1] = R.from_quat([float(pose[4]), float(pose[5]),
                                                    float(pose[6]), float(pose[7])]).as_matrix()
    pose_updated[0, -1] = float(pose[1])
    pose_updated[1, -1] = float(pose[2])
    pose_updated[2, -1] = float(pose[3])
    return pose_updated

def normalization(q):
    return q / (np.sum(q * q))**0.5

def generate_pose_with_noise(pose_gt, noise_file, edges_file, type):
    initialization_flag = False
    last_pose = None
    last_pose_noise = None
    id = 0
    for pose in pose_gt:
        pose = pose.split(" ")
        noise_file.write(pose[0])
        if not initialization_flag:
            initialization_flag = True
            for i in range(1, len(pose)):
                noise_file.write(" " + pose[i])
            last_pose = quat2mat(pose)
            last_pose_noise = quat2mat(pose)
            continue

        current_pose = quat2mat(pose)
        relative_pose = np.matmul(np.linalg.inv(last_pose), current_pose)
        relative_pose_noise = np.identity(4)
        relative_quat = R.from_matrix(relative_pose[:-1, :-1]).as_quat()
        x_noise = float(relative_pose[0, -1]) + random.gauss(mu_translation, sigma_translation)
        y_noise = float(relative_pose[1, -1]) + random.gauss(mu_translation, sigma_translation)
        z_noise = float(relative_pose[2, -1]) + random.gauss(mu_translation, sigma_translation)
        relative_quat_noise = deepcopy(relative_quat)
        for i in range(len(relative_quat)):
            relative_quat_noise[i] += random.gauss(mu_rotation, sigma_rotation)
        relative_quat_noise = normalization(relative_quat_noise)
        relative_pose_noise[0, -1] = x_noise
        relative_pose_noise[1, -1] = y_noise
        relative_pose_noise[2, -1] = z_noise
        relative_pose_noise[:-1, :-1] = R.from_quat(relative_quat_noise).as_matrix()
        current_pose_noise = np.matmul(last_pose_noise, relative_pose_noise)
        currnet_quat_noise = R.from_matrix(current_pose[:-1, :-1]).as_quat()

        edges_file.write(str(type) + " " + str(id) + " " + str(id+1) + " " + str(x_noise) + 
        " " + str(y_noise) + " " + str(z_noise) + " " + str(relative_quat_noise[0]) + " " 
        + str(relative_quat_noise[1]) + " " + str(relative_quat_noise[2]) + " " + str(relative_quat_noise[3]) + "\n")

        noise_file.write(" " + str(current_pose_noise[0, -1]) + " " + str(current_pose_noise[1, -1])  + " " + 
        str(current_pose_noise[2, -1]) + " " + str(currnet_quat_noise[0]) + " " + str(currnet_quat_noise[1])
        + " " + str(currnet_quat_noise[2]) + " " + str(currnet_quat_noise[3]) + "\n")

        id += 1
        last_pose = current_pose
        last_pose_noise = current_pose_noise

def add_edges(type, edges_file, posesA, posesB, seqA, seqB = None):
    if seqB != None:
        if len(seqA ) != len(seqB):
            print("One-to-one correspondence is not satisfied.")
            return
    if seqB == None:
        seqB = list(np.zeros(len(seqA)))
    for i in range(len(seqA)):
        poseA = posesA[seqA[i]].split(" ")
        poseA_mat = quat2mat(poseA)
        poseB = posesB[int(seqB[i])].split(" ")
        poseB_mat = quat2mat(poseB)
        relative_pose_mat = np.matmul(np.linalg.inv(poseA_mat), poseB_mat)
        relative_quat_noise = R.from_matrix(relative_pose_mat[:-1, :-1]).as_quat()
        for j in range(len(relative_quat_noise)):
            relative_quat_noise[j] += 0
        x_noise = float(relative_pose_mat[0, -1])
        y_noise = float(relative_pose_mat[1, -1])
        z_noise = float(relative_pose_mat[2, -1])

        edges_file.write(
        str(type) + " " + str(seqA[i]) + " " + str(int(seqB[i])) + " " + str(x_noise) + " " + str(y_noise) + " " + 
        str(z_noise) + " " + str(relative_quat_noise[0]) + " " + str(relative_quat_noise[1]) + " " + 
        str(relative_quat_noise[2]) + " " + str(relative_quat_noise[3]) + "\n")

        
if __name__ == '__main__':
    poseA_quaterniond = open("../data/poseA_quaterniond.txt", "r").readlines()
    poseA_noise = open("../data/poseA_noise.txt", "w")
    poseB_quaterniond = open("../data/poseB_quaterniond.txt", "r").readlines()
    poseB_noise = open("../data/poseB_noise.txt", "w")
    edges = open("../data/edges.txt", "a")
    edges.truncate(0)

    generate_pose_with_noise(poseA_quaterniond, poseA_noise, edges, 1)
    generate_pose_with_noise(poseB_quaterniond, poseB_noise, edges, 2)
    setA = [i for i in range(31, 40)]
    setB = [j for j in range(20, 29)]
    add_edges(0, edges, poseA_quaterniond, poseB_quaterniond, setA, setB)
    setA = [i for i in range(2,8)] + [i for i in range(85, 91)]
    add_edges(1, edges, poseA_quaterniond, poseA_quaterniond, setA)
    setB = [i for i in range(2,8)] + [i for i in range(54, 59)]
    add_edges(2, edges, poseB_quaterniond, poseB_quaterniond, setB)