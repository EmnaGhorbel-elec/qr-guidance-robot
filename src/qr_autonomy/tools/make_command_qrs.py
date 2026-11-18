#!/usr/bin/env python3
"""Print-ready command QR codes.

  python3 make_command_qrs.py commands.txt out_dir [--side-cm 15]

commands.txt: one command per line (see qr_autonomy/command_parser.py), '#' = comment.
  C01|TURN_L|60
  C02|MOVE|1.0
  C03|FOLLOW
  C04|STOP

Writes out_dir/<id>.png (QR only) and out_dir/sheet.pdf (one A4 page per QR, QR printed at
exactly --side-cm centimetres incl. a 4-module quiet zone, label underneath for you).
The physical side must match what you used to measure servo_k (README_AUTONOMY.md, step 3).
Needs opencv (cv2.QRCodeEncoder) and Pillow. Every text is checked by decoding it back.
"""
import argparse
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from qr_autonomy.command_parser import parse_command  # noqa: E402


def encode(text, px_per_module=12):
    enc = cv2.QRCodeEncoder.create()
    qr = enc.encode(text)                      # includes a small border already
    qr = cv2.resize(qr, None, fx=px_per_module, fy=px_per_module, interpolation=cv2.INTER_NEAREST)
    return cv2.copyMakeBorder(qr, 4 * px_per_module, 4 * px_per_module, 4 * px_per_module,
                              4 * px_per_module, cv2.BORDER_CONSTANT, value=255)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('commands')
    ap.add_argument('out_dir')
    ap.add_argument('--side-cm', type=float, default=15.0)
    ap.add_argument('--dpi', type=int, default=300)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    lines = [ln.strip() for ln in open(a.commands) if ln.strip() and not ln.strip().startswith('#')]
    pages = []
    ids = set()
    for text in lines:
        cmd = parse_command(text)                 # refuses an invalid command before printing it
        if cmd.cid in ids:
            sys.exit(f'duplicate id {cmd.cid}: every QR must have a UNIQUE id')
        ids.add(cmd.cid)
        img = encode(text)
        back, _, _ = cv2.QRCodeDetector().detectAndDecode(img)
        if back != text:
            sys.exit(f'{text!r}: decode check failed ({back!r})')
        cv2.imwrite(os.path.join(a.out_dir, f'{cmd.cid}.png'), img)
        side_px = int(round(a.side_cm / 2.54 * a.dpi))
        qr = cv2.resize(img, (side_px, side_px), interpolation=cv2.INTER_NEAREST)
        page = Image.new('L', (int(21.0 / 2.54 * a.dpi), int(29.7 / 2.54 * a.dpi)), 255)
        page.paste(Image.fromarray(qr), ((page.width - side_px) // 2, int(2.5 / 2.54 * a.dpi)))
        draw = ImageDraw.Draw(page)
        try:
            font = ImageFont.load_default(size=60)
        except TypeError:
            font = ImageFont.load_default()
        y = int(2.5 / 2.54 * a.dpi) + side_px + 80
        draw.text((page.width // 2, y), text, fill=0, font=font, anchor='mm')
        draw.text((page.width // 2, y + 90), f'{a.side_cm:g} cm square (with quiet zone)',
                  fill=100, font=font, anchor='mm')
        pages.append(page)
        print(f'{cmd.cid}: {cmd.actions}')
    if pages:
        pages[0].save(os.path.join(a.out_dir, 'sheet.pdf'), save_all=True,
                      append_images=pages[1:], resolution=a.dpi)
    print(f'{len(pages)} QR written to {a.out_dir}')


if __name__ == '__main__':
    main()
