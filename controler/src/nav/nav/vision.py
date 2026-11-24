import cv2
import numpy as np

def extract_centerline(mask):
    row_x, row_y = [], []
    for y, row in enumerate(mask):
        xs = np.flatnonzero(row)
        if len(xs) > 0:
            row_x.append(xs.mean())
            row_y.append(y)
    
    return (
        np.array(row_y, dtype=np.float64),
        np.array(row_x, dtype=np.float64)
    )

def fit_line_polynomial(row_y, row_x, degree=2):
    if len(row_y) < degree + 2:
        return None
    coefficients = np.polyfit(row_y, row_x, degree)
    return np.poly1d(coefficients)

def draw_polynomial(vis, poly, color=(0, 0, 255)):
    ys = np.arange(vis.shape[0], dtype=np.float64)
    xs = np.clip(poly(ys), 0, vis.shape[1] - 1)
    points = (
        np.column_stack((xs, ys))
        .astype(np.int32)
        .reshape(-1, 1, 2)
    )
    
    cv2.polylines(vis, [points], False, color, 1)

def polynomial_errors(poly, image_width, y_eval):
    lateral_error = poly(y_eval) - image_width / 2
    heading_error = np.arctan(poly.deriv()(y_eval))
    return lateral_error, heading_error

def process(frame):
    frame = cv2.resize(frame, (100, 100))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (3,3), 0)

    h, w = frame.shape[0], frame.shape[1]

    start = (0, int(h * 0.4))
    end = (w, h)

    roi = frame[start[1]:end[1], start[0]: end[0]] 

    _, mask = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)
    mask *= 1
    mask = cv2.GaussianBlur(mask, (3,3), 0)

    row_y, row_x = extract_centerline(mask)
    poly = fit_line_polynomial(row_y, row_x, degree=2)

    vis = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    if poly is not None:
        draw_polynomial(vis, poly)
        y_eval = mask.shape[0] * 0.5
        lateral_error, heading_error = polynomial_errors(poly, w, y_eval)
    else:
        lateral_error = 0
        heading_error = 0

    return vis, mask, lateral_error, heading_error