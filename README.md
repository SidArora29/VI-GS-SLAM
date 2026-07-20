# VI-GS-SLAM: Visual-Inertial Gaussian-Splatting SLAM

Fusing a from-scratch stereo VIO system (GTSAM factor graphs + IMU preintegration + ISAM2) with a 3D Gaussian-Splatting mapper (built on [MonoGS](https://github.com/muskie82/MonoGS)) — replacing MonoGS's own photometric pose-tracker with externally-supplied VIO poses, to directly test how pose accuracy affects Gaussian-map reconstruction quality.

This repo is the integration layer. It depends on two companion repos:
- [`gtsam-custom-vio`]([https://github.com/SidArora29/gtsam-custom-vio](https://github.com/SidArora29/Custom-Stereo-Visual-Inertial-Odometry)) — the VIO front-end that produces the trajectory files consumed here.
- A patched fork of [MonoGS](https://github.com/muskie82/MonoGS) — the mapping back-end, modified to accept external poses ([diff/patch included below](#patches-applied-to-monogs)).

## Why

MonoGS estimates camera pose and builds the Gaussian map jointly, using a photometric-loss-driven optimizer for tracking. That works well in general but is known to struggle in low-texture, fast-motion conditions — exactly where a factor-graph VIO (fusing IMU + stereo) should be more robust, since it isn't relying on photometric consistency alone. This project decouples the two: **fix the camera trajectory from an independently-run VIO pipeline, and let MonoGS optimize only the map**, isolating how much of final reconstruction quality is actually gated by pose accuracy.

## What was actually found

Tested on EuRoC `MH_01_easy` (stereo + IMU, Machine Hall sequence):

| Pose source | ATE RMSE (vs. mocap ground truth, `evo_ape -a`) | PSNR (0–600 frame window, MonoGS Gaussian map) |
|---|---|---|
| MonoGS native photometric tracker | — | 23.6 dB |
| This project's VIO (classical stereo KLT + GTSAM) | 0.998 m | 13.3 dB |
| This project's VIO (SuperPoint+LightGlue frontend) | 1.43 m | 11.4 dB |

**Finding:** VIO pose accuracy directly gates Gaussian-map quality — a consistent 5–10 dB PSNR gap between VIO-driven and native-tracker-driven mapping, reproduced across multiple trajectory windows. The classical stereo frontend outperformed a SuperPoint+LightGlue learned frontend on both trajectory accuracy and final map quality on this dataset, a result confirmed by direct A/B comparison, not assumption.

**Loop closure:** implemented (ORB feature matching + PnP-verified geometric closure, inserted as robust `BetweenFactorPose3` constraints into the live ISAM2 graph). On this dataset it **degraded** trajectory accuracy — diagnosed as a perceptual-aliasing failure: Machine Hall's repetitive structural geometry (girders, scaffolding) produced a burst of mutually-reinforcing false-positive closures that a robust kernel couldn't reject, since the false matches agreed with each other. Reverted; documented here as a known failure mode rather than hidden. The fix (inlier-ratio thresholding, per-candidate deduplication, multi-hypothesis agreement) is scoped but not yet implemented — see [Future Work](#future-work).

## Architecture

```
Stereo images + IMU  →  GTSAM factor graph (VIO)  →  trajectory (.tum)
                                                            │
                                                            ▼
                                          MonoGS dataset loader (patched)
                                          reads external pose per frame,
                                          skips photometric tracking,
                                          optimizes only Gaussian params
                                                            │
                                                            ▼
                                              Rendered Gaussian map + PSNR/ATE
```

## Patches applied to MonoGS

- `utils/dataset.py` — `EuRoCParser` accepts `use_external_pose` / `external_pose_file` config flags; loads poses from a TUM-format trajectory instead of EuRoC's mocap ground truth when enabled.
- `utils/slam_frontend.py` — `tracking()` gated behind `use_external_pose`: when enabled, commits the loaded pose directly (`viewpoint.update_RT(R_gt, T_gt)`) and does a single no-grad render pass, instead of running the Adam photometric pose-optimizer.
- `slam.py` — added pose-count sanity check + logging (`[VIGS-SLAM] Loading external VIO poses from ...`) so it's always visible which pose source a given run actually used.

Full diffs in [`patches/`](./patches).

## Reproducing

```bash
git clone https://github.com/SidArora29/vi-gs-slam
cd vi-gs-slam
# follow setup.md for MonoGS submodule build (simple-knn, diff-gaussian-rasterization)

# 1. generate a VIO trajectory (see gtsam-custom-vio repo)
# 2. point configs/stereo/euroc/mh01.yaml at it:
#      Training.use_external_pose: True
#      Training.external_pose_file: /path/to/your_vio_trajectory.tum
python slam.py --config configs/stereo/euroc/mh01.yaml --eval
```

## Future Work

- Fix loop closure with inlier-ratio thresholding + per-candidate dedup + second-candidate agreement, retest on the same sequence.
- Investigate the fixed frontend-yield gap directly (stereo match count per frame) rather than only its downstream effect on ATE/PSNR.
- Extend to a sequence with genuine loop revisits away from repetitive structure, to separate the aliasing failure mode from loop closure's general viability on this pipeline.

## Related

- [`gtsam-custom-vio`]([https://github.com/SidArora29/Custom-Stereo-Visual-Inertial-Odometry](https://github.com/SidArora29/Custom-Stereo-Visual-Inertial-Odometry)) — the standalone VIO system (factor graph, IMU preintegration, both classical and learned frontends).
- [MonoGS](https://github.com/muskie82/MonoGS) — Matsuki et al., CVPR 2024, the mapping backend this project builds on.
