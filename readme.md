# Fundamental Matrix from Scratch

A hand-rolled implementation of the **normalized 8-point algorithm** for estimating
the fundamental matrix between two images, written in C++ with OpenCV. Built to learn
classical computer vision and C++ from the ground up — the linear algebra (building the
constraint matrix, the two SVDs, rank-2 enforcement) is implemented by hand rather than
calling `cv::findFundamentalMat`.

## What it does

Given two photos of the same scene from different viewpoints, it:

1. Detects and matches features (ORB + brute-force Hamming matching)
2. Computes the fundamental matrix `F` via the normalized 8-point algorithm
3. Verifies the result against OpenCV and by epipolar constraint error
4. Filters RANSAC inliers using **Sampson distance** — a first-order geometric
   error that weights each correspondence by how far its matched point lies from
   the predicted epipolar line, without requiring full triangulation:
   `d_S = (x'ᵀFx)² / ((Fx)₀² + (Fx)₁² + (Fᵀx')₀² + (Fᵀx')₁²)`
5. Draws the epipolar lines to confirm the geometry visually

`F` encodes the epipolar geometry between the two views: for corresponding points
`x` and `x'`, it satisfies `x'ᵀ F x = 0`.

## Build

Requires OpenCV and CMake.

```bash
mkdir build && cd build
cmake ..
make
cd ..
```

## Run

Run from the project root so the `data/` image paths resolve:

```bash
./build/00_load        # load + display the image pair
./build/01_match       # detect + match features, draw matches
./build/02_opencv_f    # reference F via cv::findFundamentalMat + epipolar lines
./build/03_my_f        # hand-rolled 8-point F with RANSAC + Sampson distance
./build/04_calibrate   # calibrate camera intrinsics from checkerboard images
```

## Project layout

```
apps/          one small program per build step (00–04)
  00_load        load + display image pair
  01_match       detect + match features
  02_opencv_f    reference F via cv::findFundamentalMat
  03_my_f        hand-rolled 8-point F with RANSAC
  04_calibrate   camera calibration from a checkerboard image set
include/
  epipolar_viz.hpp   static helpers for drawing epipolar lines
  fundamental.hpp    FundamentalMatrix class (compute, ransac, epipolarError)
data/
  left.jpg / right.jpg         stereo pair 1
  left_1.jpg / right_1.jpg     stereo pair 2
  calibration_imgs/            checkerboard images used for intrinsic calibration
camera_calibation.yml          saved camera intrinsics + distortion coefficients
CMakeLists.txt
```

## The algorithm (8-point, in order)

1. **Normalize** points (center at origin, scale mean distance to √2) — critical for
   numerical stability
2. **Build** the n×9 constraint matrix `A`, one row per correspondence
3. **SVD #1** — solve `Af = 0`; the solution is the singular vector with the smallest
   singular value, reshaped to 3×3
4. **SVD #2** — zero the smallest singular value to enforce rank 2 (makes all epipolar
   lines meet at a single epipole)
5. **Denormalize** — undo the normalization so `F` works in pixel coordinates

## Results

### Image pair 1 (left.jpg / right.jpg)

**OpenCV reference (`cv::findFundamentalMat` + RANSAC internally)**
![OpenCV epipolar lines on scene](output/ocv_f_scene.png)

**Hand-rolled 8-point algorithm (all matches, no RANSAC)**
![My F epipolar lines on scene](output/my_f_scene.png)

**Hand-rolled 8-point algorithm with RANSAC + Sampson distance**
![RANSAC epipolar lines on scene](output/ransac_f_scene.png)


---

### Image pair 2 (left_1.jpg / right_1.jpg)

**OpenCV reference**
![OpenCV epipolar lines on scene 2](output/ocv_f_scene2.png)

**Hand-rolled 8-point algorithm (all matches, no RANSAC)**
![My F epipolar lines on scene 2](output/my_f_scene2.png)

**Hand-rolled 8-point algorithm with RANSAC + Sampson distance**
![RANSAC epipolar lines on scene 2](output/ransac_f_scene2.png)

# Camera Motion and Written Results
As seen above an indoor room (lots of repetitive
texture — serape stripes, blind slats, curtain grid) and an outdoor stone wall
(rich non-repetitive texture). In both the camera was roughly translated sideway with no rotation, so the epipolar lines should be near-horizontal. Three fundamental matrices are compared, each visualized by drawing epipolar
lines (green) and the corresponding points (red) on the right image. A correct F
is indicated by red points lying on their green lines. 

generally the raw algebraic 8-point fit can satisfy the epipolar constraint numerically
while still encoding an incorrect rotation. Switching the inlier metric to the Sampson distance, combined with RANSAC outlier rejection, brings the from-scratch
estimate into a closer agreement with OpenCV and with the true camera motion.


## Camera Calibration Results

Intrinsics estimated from a set of checkerboard images using `04_calibrate`
(`cv::calibrateCamera` with a 9×6 board). Results saved to `camera_calibation.yml`.

**RMS reprojection error: 0.2533 px**

**Camera matrix K:**

```
[1541.344  0.000     746.985]
[0.000     1537.615  988.655]
[0.000     0.000     1.000  ]
```

| Parameter | Value |
|-----------|-------|
| fx | 1541.344 px |
| fy | 1537.615 px |
| cx | 746.985 px |
| cy | 988.655 px |

**Distortion coefficients (k1, k2, p1, p2, k3):**

| k1 | k2 | p1 | p2 | k3 |
|----|----|----|----|----|
| 0.2913 | −1.9516 | −0.000771 | −0.000768 | 3.9679 |

The large k3 and opposing k1/k2 signs are typical for a wide-angle lens correcting
strong barrel distortion at mid-radii. The sub-pixel RMS error confirms the board
detections are consistent across the calibration set.

## Notes / next steps

- **Essential matrix** (`04_essential`) — lift F to E using iPhone camera intrinsics K via `E = Kᵀ F K`, decompose E via SVD into (R, t), resolve the 4-way sign ambiguity with a cheirality check
- **Triangulation** — given P1 = K[I|0] and P2 = K[R|t], use DLT on each RANSAC inlier pair to recover a sparse 3D point cloud
- **Dense depth** — if images are a rectified stereo pair, `cv::StereoSGBM` can produce a full disparity map
