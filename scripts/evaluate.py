# 评测APE、RPE、FPS等指标的脚本
import numpy as np
import sys

def load_traj(path):
    poses = []
    with open(path) as f:
        for line in f:
            nums = list(map(float, line.strip().split()))
            pose = np.eye(4)
            pose[:3,:4] = np.array(nums).reshape((3,4))
            poses.append(pose)
    return np.array(poses)

def compute_ape(est, gt):
    errors = []
    for e, g in zip(est, gt):
        p_e = e[:3,3]
        p_g = g[:3,3]
        errors.append(np.linalg.norm(p_e - p_g))
    return np.mean(errors), np.std(errors)

if __name__ == "__main__":
    est = load_traj(sys.argv[1])
    gt = load_traj(sys.argv[2])
    min_len = min(len(est), len(gt))
    est, gt = est[:min_len], gt[:min_len]

    mean_ape, std_ape = compute_ape(est, gt)
    print("APE: %.4f ± %.4f meters" % (mean_ape, std_ape))
