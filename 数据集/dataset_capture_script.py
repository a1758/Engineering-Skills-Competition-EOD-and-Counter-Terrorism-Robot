# Dataset Capture Script - By: zjq - Sun Oct 11 2026

# Use this script to control how your OpenMV Cam captures images for your
# dataset. You should apply the same image pre-processing steps you expect
# to run on images that you will feed to your model during run-time.

import csi
import time

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)  # Modify as you like.
csi0.framesize(csi.QVGA)  # Modify as you like.
csi0.snapshot(time=2000)

clock = time.clock()

while True:
    clock.tick()
    img = csi0.snapshot()
    # Apply lens correction if you need it.
    # img.lens_corr()
    # Apply rotation correction if you need it.
    # img.rotation_corr()
    # Apply other filters...
    # E.g. mean/median/mode/midpoint/etc.
    print(clock.fps())
