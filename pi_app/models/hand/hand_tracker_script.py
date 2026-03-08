"""
On-device Script node code for the OAK-D hand-tracking pipeline.

Ported from geaxgx/depthai_hand_tracker (template_manager_script_solo.py).
Runs on the MyriadX VPU in Edge Solo mode -- no Body Pre Focusing, no XYZ,
no world landmarks.  Coordinates the palm-detection → crop → hand-landmark
two-stage pipeline entirely on-device and sends only the 21 landmark
coordinates (~2 KB/frame) back to the host via marshal-serialised buffers.

Placeholder tokens (_PAD_H, _IMG_H, etc.) are substituted at pipeline
build time by ``build_hand_tracker_script()`` in oak_depth.py.
"""
import marshal
from math import sin, cos, atan2, pi, degrees, floor

pad_h = _PAD_H
img_h = _IMG_H
img_w = _IMG_W
frame_size = _FRAME_SIZE
crop_w = _CROP_W
pd_score_thresh = _PD_SCORE_THRESH
lm_score_thresh = _LM_SCORE_THRESH

class BufferMgr:
    def __init__(self):
        self._bufs = {}
    def __call__(self, size):
        try:
            buf = self._bufs[size]
        except KeyError:
            buf = self._bufs[size] = Buffer(size)
        return buf

buffer_mgr = BufferMgr()

def send_result(result):
    result_serial = marshal.dumps(result)
    buffer = buffer_mgr(len(result_serial))
    buffer.getData()[:] = result_serial
    node.io['host'].send(buffer)

def send_result_no_hand(pd_inf, nb_lm_inf):
    send_result(dict([("pd_inf", pd_inf), ("nb_lm_inf", nb_lm_inf)]))

def send_result_hand(pd_inf, nb_lm_inf, lm_score, handedness, rect_center_x, rect_center_y, rect_size, rotation, rrn_lms, sqn_lms):
    send_result(dict([
        ("pd_inf", pd_inf), ("nb_lm_inf", nb_lm_inf),
        ("lm_score", [lm_score]), ("handedness", [handedness]),
        ("rotation", [rotation]),
        ("rect_center_x", [rect_center_x]), ("rect_center_y", [rect_center_y]),
        ("rect_size", [rect_size]),
        ("rrn_lms", [rrn_lms]), ("sqn_lms", [sqn_lms]),
    ]))

def rr2img(rrn_x, rrn_y):
    X = sqn_rr_center_x + sqn_rr_size * ((rrn_x - 0.5) * cos_rot + (0.5 - rrn_y) * sin_rot)
    Y = sqn_rr_center_y + sqn_rr_size * ((rrn_y - 0.5) * cos_rot + (rrn_x - 0.5) * sin_rot)
    return X, Y

def normalize_radians(angle):
    return angle - 2 * pi * floor((angle + pi) / (2 * pi))

def build_lm_config():
    """Construct an ImageManipConfig for the current hand ROI."""
    rr = RotatedRect()
    rr.center.x = sqn_rr_center_x
    rr.center.y = (sqn_rr_center_y * frame_size - pad_h) / img_h
    rr.size.width = sqn_rr_size
    rr.size.height = sqn_rr_size * frame_size / img_h
    rr.angle = degrees(rotation)
    cfg = ImageManipConfig()
    cfg.addCropRotatedRect(rr, True)
    cfg.setOutputSize(lm_input_size, lm_input_size, ImageManipConfig.ResizeMode.STRETCH)
    cfg.setFrameType(ImgFrame.Type.BGR888p)
    return cfg

send_new_frame_to_branch = 1

id_wrist = 0
id_index_mcp = 5
id_middle_mcp = 9
id_ring_mcp = 13
ids_for_bounding_box = [0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18]
lm_input_size = 224

while True:
    nb_lm_inf = 0
    if send_new_frame_to_branch == 1:
        detection = node.io['from_post_pd_nn'].get().getLayerFp16("result")
        pd_score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y = detection[:8]

        if pd_score < pd_score_thresh or box_size < 0:
            send_result_no_hand(True, 0)
            send_new_frame_to_branch = 1
            continue

        kp02_x = kp2_x - kp0_x
        kp02_y = kp2_y - kp0_y
        sqn_rr_size = 2.9 * box_size
        rotation = 0.5 * pi - atan2(-kp02_y, kp02_x)
        rotation = normalize_radians(rotation)
        sqn_rr_center_x = box_x + 0.5 * box_size * sin(rotation)
        sqn_rr_center_y = box_y - 0.5 * box_size * cos(rotation)

    node.io['pre_lm_manip_cfg'].send(build_lm_config())
    nb_lm_inf += 1

    lm_result = node.io['from_lm_nn'].get()
    lm_score = lm_result.getLayerFp16("Identity_1")[0]
    if lm_score > lm_score_thresh:
        handedness = lm_result.getLayerFp16("Identity_2")[0]
        rrn_lms = lm_result.getLayerFp16("Identity_dense/BiasAdd/Add")
        sqn_lms = []
        cos_rot = cos(rotation)
        sin_rot = sin(rotation)
        for i in range(21):
            rrn_lms[3 * i] /= lm_input_size
            rrn_lms[3 * i + 1] /= lm_input_size
            rrn_lms[3 * i + 2] /= lm_input_size
            sqn_x, sqn_y = rr2img(rrn_lms[3 * i], rrn_lms[3 * i + 1])
            sqn_lms += [sqn_x, sqn_y]

        send_result_hand(
            send_new_frame_to_branch == 1, nb_lm_inf,
            lm_score, handedness,
            sqn_rr_center_x, sqn_rr_center_y, sqn_rr_size, rotation,
            rrn_lms, sqn_lms,
        )
        send_new_frame_to_branch = 2

        x0 = sqn_lms[0]
        y0 = sqn_lms[1]
        x1 = 0.25 * (sqn_lms[2 * id_index_mcp] + sqn_lms[2 * id_ring_mcp]) + 0.5 * sqn_lms[2 * id_middle_mcp]
        y1 = 0.25 * (sqn_lms[2 * id_index_mcp + 1] + sqn_lms[2 * id_ring_mcp + 1]) + 0.5 * sqn_lms[2 * id_middle_mcp + 1]
        rotation = 0.5 * pi - atan2(y0 - y1, x1 - x0)
        rotation = normalize_radians(rotation)
        min_x = min_y = 1
        max_x = max_y = 0
        for id in ids_for_bounding_box:
            min_x = min(min_x, sqn_lms[2 * id])
            max_x = max(max_x, sqn_lms[2 * id])
            min_y = min(min_y, sqn_lms[2 * id + 1])
            max_y = max(max_y, sqn_lms[2 * id + 1])
        axis_aligned_center_x = 0.5 * (max_x + min_x)
        axis_aligned_center_y = 0.5 * (max_y + min_y)
        cos_rot = cos(rotation)
        sin_rot = sin(rotation)
        min_x = min_y = 1
        max_x = max_y = -1
        for id in ids_for_bounding_box:
            original_x = sqn_lms[2 * id] - axis_aligned_center_x
            original_y = sqn_lms[2 * id + 1] - axis_aligned_center_y
            projected_x = original_x * cos_rot + original_y * sin_rot
            projected_y = -original_x * sin_rot + original_y * cos_rot
            min_x = min(min_x, projected_x)
            max_x = max(max_x, projected_x)
            min_y = min(min_y, projected_y)
            max_y = max(max_y, projected_y)
        projected_center_x = 0.5 * (max_x + min_x)
        projected_center_y = 0.5 * (max_y + min_y)
        center_x = projected_center_x * cos_rot - projected_center_y * sin_rot + axis_aligned_center_x
        center_y = projected_center_x * sin_rot + projected_center_y * cos_rot + axis_aligned_center_y
        width = max_x - min_x
        height = max_y - min_y
        sqn_rr_size = 2 * max(width, height)
        sqn_rr_center_x = center_x + 0.1 * height * sin_rot
        sqn_rr_center_y = center_y - 0.1 * height * cos_rot
    else:
        send_result_no_hand(send_new_frame_to_branch == 1, nb_lm_inf)
        send_new_frame_to_branch = 1
