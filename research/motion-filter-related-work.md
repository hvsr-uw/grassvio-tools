# Motion-filter related work

I checked these primary author and publisher summaries on November 19,
2026. They show where the motion-filter idea overlaps with earlier work.

| Work | What the primary source establishes | Implication for our work |
|---|---|---|
| [DynaVINS, Song et al., 2022](https://arxiv.org/abs/2208.11500) | Uses IMU-preintegration pose priors in robust bundle adjustment to reject dynamic features, and addresses temporarily static objects in loop closure. | An IMU prior for rejecting dynamic features is prior art. A vegetation contribution needs evidence beyond that general idea. |
| [DynaSLAM, Bescos et al., 2018](https://bertabescos.github.io/DynaSLAM/) | Adds moving-object detection using multi-view geometry, deep learning, or both, plus background inpainting, to ORB-SLAM2. | Geometry-based dynamic rejection is prior art. We need to evaluate the hard case of distributed, deforming vegetation instead of describing generic rejection as new. |
| [BANSAC, Piedade and Miraldo, ICCV 2023](https://openaccess.thecvf.com/content/ICCV2023/html/Piedade_BANSAC_A_Dynamic_BAyesian_Network_for_Adaptive_SAmple_Consensus_ICCV_2023_paper.html) | Updates individual inlier scores with a dynamic Bayesian network during RANSAC, uses them for weighted sampling and a stopping criterion. | The new implementation uses temporal pixel-residual filtering before RANSAC. It does not implement BANSAC and does not claim a Bayesian probability of being rigid. |

I am testing persistent-window residuals on vegetation, with
controlled camera/wind factors and pose accuracy measured on separate
frames. The synthetic 2x2 checks the implementation. I still need controlled
wind videos, independent camera-motion evidence and real-video comparisons
before I can say what is new.

I also checked [IEEE Robotics and Automation Letters](https://www.ieee-ras.org/publications/ieee-robotics-and-automation-letters/).
Its official page describes journal publication and a separate opportunity
for accepted papers to be presented at eligible RAS conferences. It lists
no conference-specific RA-L submission deadlines and gives a transfer window
within 270 days of acceptance. I would need to check the current transfer
window once the experiments support a manuscript.
