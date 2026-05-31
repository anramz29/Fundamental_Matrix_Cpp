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
4. Draws the epipolar lines to confirm the geometry visually

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
./build/03_my_f        # hand-rolled 8-point F, compared against OpenCV
```

## Project layout

```
apps/        one small program per build step (00–03)
data/        the stereo image pair (left.jpg, right.jpg)
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

**OpenCV reference (`cv::findFundamentalMat` + RANSAC internally)**
![OpenCV epipolar lines](output/ocv_f.png)

 Lines are horizontal and converge to an epipole far off to the right side, outside the frame. Nearly-parallel horizontal lines.


**Hand-rolled 8-point algorithm (all matches, no RANSAC)**
![My F epipolar lines](output/my_f.png)

Here we can notice that the dot's are off the line in several places, suggesting a corrupted fit.

**Hand-rolled 8-point algorithm with RANSAC**
![RANSAC epipolar lines](output/ransac_f.png)

Here we see dot's are on the line but the lines converged close to a epipole at the tree, a valid fit but claims a different camera motion than open_cv

## Notes / next steps

- The current version uses all matches, including outliers from repeated texture, so the
  estimate is pulled off-true. Adding **RANSAC** (run the 8-point on random minimal
  subsets, keep the F with the most inliers) is the next step to handle outliers — the
  same thing `cv::findFundamentalMat` does internally.