#!/usr/bin/env python3
# Compares VO/VIO/SLAM position+orientation estimates against EuRoC ground truth.
import csv
import math
import sys

DATASET = "src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy/mav0"
BUILD = "src/build"


def load_rows(path, pos_cols, quat_cols):
    # generic loader: each source file lays out timestamp/position/quaternion in different
    # column positions (vio_estimate.csv has velocity between position and quaternion, etc.)
    rows = []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            t = float(row[0])
            p = [float(row[c]) for c in pos_cols]
            q = [float(row[c]) for c in quat_cols]
            rows.append((t, p, q))
    return rows


def quat_angle_deg(q1, q2):
    dot = sum(a * b for a, b in zip(q1, q2))
    dot = max(-1.0, min(1.0, abs(dot)))  # abs() handles the q/-q double-cover
    return math.degrees(2 * math.acos(dot))


def nearest_match(estimates, ground_truth, tol_ns=5e6):
    # both lists sorted by timestamp ascending; two-pointer nearest-neighbor match
    pairs = []
    j = 0
    n = len(ground_truth)
    for t, p, q in estimates:
        while j + 1 < n and abs(ground_truth[j + 1][0] - t) <= abs(ground_truth[j][0] - t):
            j += 1
        gt_t, gt_p, gt_q = ground_truth[j]
        if abs(gt_t - t) > tol_ns:
            continue
        pairs.append((t, p, q, gt_p, gt_q))
    return pairs


def report(name, pairs):
    if not pairs:
        print(f"{name}: no matched samples")
        return
    pos_errs = [math.sqrt(sum((a - b) ** 2 for a, b in zip(p, gt_p))) for _, p, _, gt_p, _ in pairs]
    ang_errs = [quat_angle_deg(q, gt_q) for _, _, q, _, gt_q in pairs]

    def stats(vals):
        n = len(vals)
        mean = sum(vals) / n
        rmse = math.sqrt(sum(v * v for v in vals) / n)
        return mean, rmse, max(vals)

    pm, pr, px = stats(pos_errs)
    am, ar, ax = stats(ang_errs)
    print(f"{name}: {len(pairs)} matched samples")
    print(f"  position error (m):      mean={pm:.3f} rmse={pr:.3f} max={px:.3f}  (first={pos_errs[0]:.3f}, last={pos_errs[-1]:.3f})")
    print(f"  orientation error (deg): mean={am:.2f} rmse={ar:.2f} max={ax:.2f}  (first={ang_errs[0]:.2f}, last={ang_errs[-1]:.2f})")


def main():
    vicon_gt = load_rows(f"{DATASET}/vicon0/data.csv", [1, 2, 3], [4, 5, 6, 7])
    body_gt = load_rows(f"{DATASET}/state_groundtruth_estimate0/data.csv", [1, 2, 3], [4, 5, 6, 7])

    # vo_estimate.csv: timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z
    vo = load_rows(f"{BUILD}/vo_estimate.csv", [1, 2, 3], [4, 5, 6, 7])
    # vio_estimate.csv: timestamp,p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,ba...,bg...
    vio = load_rows(f"{BUILD}/vio_estimate.csv", [1, 2, 3], [7, 8, 9, 10])
    # slam_trajectory.csv: timestamp,frame_id,p_x,p_y,p_z,q_w,q_x,q_y,q_z
    slam = load_rows(f"{BUILD}/slam_trajectory.csv", [2, 3, 4], [5, 6, 7, 8])

    report("VO   (vs vicon0 marker ground truth)", nearest_match(vo, vicon_gt))
    report("VIO  (vs body ground truth)", nearest_match(vio, body_gt))
    report("SLAM (vs body ground truth)", nearest_match(slam, body_gt))


if __name__ == "__main__":
    sys.exit(main())
