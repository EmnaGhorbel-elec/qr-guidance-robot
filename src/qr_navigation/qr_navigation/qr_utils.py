"""Pure helpers (no ROS) for reading detector messages."""
import json
import math


def parse_detection(raw):
    """Parse the text published by the QR detector.

    Accepts the detector's JSON ({"id": ..., "w_px": ...}) or a bare QR id.
    Returns (qr_id, w_px); w_px is None when the message does not carry it.
    """
    raw = (raw or '').strip()
    if raw.startswith('{'):
        try:
            d = json.loads(raw)
            w = d.get('w_px')
            w = None if w is None else float(w)
            return str(d.get('id', '')).strip(), (w if w is not None and w > 0.0 else None)
        except (ValueError, TypeError, AttributeError):
            return '', None
    return raw, None


def close_enough(w_px, min_width_px):
    """True if the QR is large enough in the image to count as 'robot is at the QR'.

    Gate disabled (always True) when min_width_px <= 0 or the width is unknown.
    """
    if min_width_px <= 0.0 or w_px is None:
        return True
    return w_px >= min_width_px


def range_corrected_xy(x, y, theta, w_px, k, camera_offset, standoff, max_corr):
    """Refine a checkpoint position using the QR's apparent size.

    Pinhole model: camera-to-QR distance ~ k / w_px (k depends on camera focal length,
    QR size and QR margin, see README). The robot is `standoff` m from the QR at the
    checkpoint; if the QR looks smaller/larger the robot is further/closer, so the
    position is shifted along the checkpoint heading. The shift is clamped to
    +-max_corr m so a wrong k can never do worse than the plain table position.
    Returns (x, y). With k <= 0 or an unknown width, returns the table position.
    """
    if k <= 0.0 or not w_px or w_px <= 0.0:
        return x, y
    base_dist = k / w_px + camera_offset
    shift = max(-max_corr, min(max_corr, base_dist - standoff))   # >0: robot is behind
    return x - shift * math.cos(theta), y - shift * math.sin(theta)


def parse_detection_full(raw):
    """Detector JSON as a dict with id, cx, img_w, w_px (floats) and stamp, or None if unusable.

    stamp = capture time of the frame in s (None when the detector does not send it).
    """
    raw = (raw or '').strip()
    if not raw.startswith('{'):
        return None
    try:
        d = json.loads(raw)
        stamp = d.get('stamp')
        return {'id': str(d['id']).strip(), 'cx': float(d['cx']),
                'img_w': float(d['img_w']), 'w_px': float(d['w_px']),
                'stamp': None if stamp is None else float(stamp)}
    except (ValueError, TypeError, KeyError, AttributeError):
        return None
