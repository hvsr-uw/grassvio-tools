# Motion-filter related work

I checked these primary author and publisher summaries on November 19,
2026. They show where the motion-filter idea overlaps with earlier work.

| Work | What the primary source establishes | Implication for our work |
|---|---|---|
| [DynaVINS, Song et al., 2022](https://arxiv.org/abs/2208.11500) | Uses IMU-preintegration pose priors in robust bundle adjustment to reject dynamic features, and addresses temporarily static objects in loop closure. | An IMU prior for rejecting dynamic features is prior art. A vegetation contribution needs evidence beyond that general idea. |
| [DynaSLAM, Bescos et al., 2018](https://bertabescos.github.io/DynaSLAM/) | Adds moving-object detection using multi-view geometry, deep learning, or both, plus background inpainting, to ORB-SLAM2. | Geometry-based dynamic rejection is prior art. We need to evaluate the hard case of distributed, deforming vegetation instead of describing generic rejection as new. |
| [BANSAC, Piedade and Miraldo, ICCV 2023](https://openaccess.thecvf.com/content/ICCV2023/html/Piedade_BANSAC_A_Dynamic_BAyesian_Network_for_Adaptive_SAmple_Consensus_ICCV_2023_paper.html) | Updates individual inlier scores with a dynamic Bayesian network during RANSAC, uses them for weighted sampling and a stopping criterion. | The new implementation uses temporal pixel-residual filtering before RANSAC. It does not implement BANSAC and does not claim a Bayesian probability of being rigid. |

I also checked closer vegetation work and initialization references on
November 20, 2026. I read the repository descriptions and abstracts for
the following sources; I have not reproduced their experiments.

| Work | What I found in the primary source | What I need to distinguish |
|---|---|---|
| [Visual-inertial odometry for windblown vegetated environments, Sam Schofield, 2023 thesis](https://ir.canterbury.ac.nz/items/40210315-29c2-4ad6-9d30-c6934452f610) | The abstract describes an optical-flow outlier detector used before RANSAC in windblown forests and reports up to 20% better VO accuracy. | This is direct overlap with the broad vegetation prefilter idea. I need the method and experiment chapters before comparing its detector with the persistent-window residuals used here. |
| [Experimental Evaluation of Visual-Inertial Odometry Systems for Arable Farming, Cremona, Comelli and Pire, 2022](https://arxiv.org/abs/2206.05066v2) | Evaluates VIO on the Rosario soybean-field recordings and discusses initialization failures, tracking loss, repetitive vegetation, wind and IMU saturation. | A real vegetation evaluation is already established work. I need controlled wind and camera-motion factors rather than a general claim that vegetation is difficult. |
| [DefVINS, Cerezo and Civera, 2026 preprint](https://arxiv.org/abs/2601.00702v4) | Describes an IMU-anchored rigid component with a deformation graph, a deformable-scene benchmark and a conditioning-based update strategy. | A deformable-scene VIO claim needs comparison with this stronger model. My current filter does not estimate a deformation graph. |
| [OpenVINS inertial initializer documentation](https://docs.openvins.com/classov__init_1_1InertialInitializer.html) | Describes dynamic initialization, a calibrated static alternative, and linear estimation followed by optimization. | My nine-unknown linear initialization is a diagnostic, not a replacement for this complete initializer. A quiet first second does not by itself establish all the needed calibration. |

I now have six original-video recovery and channel-check runs, recorded in
[motion-recovery-results.md](../analysis/motion-recovery-results.md).
The synthetic 2x2 still checks the implementation. The real videos show
tracking and timing behaviour, but do not independently label wind or
provide metric camera ground truth.

I cannot claim that using camera-motion residuals before RANSAC in moving
vegetation is new. A possible contribution still needs a specific detector
difference, controlled ablations, a runtime comparison, and evaluation on
separate videos with known camera motion. I have not established that
publication claim with the current results.
