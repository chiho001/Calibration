import numpy as np
import cv2

def dof_sparse(dense_flow, grid_step=20, magnitude_thresh=1.0):
    """
    Convert dense optical flow to sparse flow using grid sampling and magnitude thresholding.

    Parameters:
        dense_flow (np.ndarray): Optical flow array of shape (H, W, 2).
        grid_step (int): Step size for the grid sampling.
        magnitude_thresh (float): Minimum magnitude to consider a flow vector valid.

    Returns:
        good_prev (np.ndarray): Coordinates in the previous frame (N, 1, 2).
        good_curr (np.ndarray): Corresponding coordinates in the current frame (N, 1, 2).
    """
    h, w = dense_flow.shape[:2]
    good_prev = []
    good_curr = []

    for y in range(0, h, grid_step):
        for x in range(0, w, grid_step):
            dx, dy = dense_flow[y, x]
            mag = np.hypot(dx, dy)
            if mag > magnitude_thresh:
                good_prev.append([[x, y]])
                good_curr.append([[x + dx, y + dy]])

    good_prev = np.array(good_prev, dtype=np.float32)
    good_curr = np.array(good_curr, dtype=np.float32)
    return good_prev, good_curr