# Motion-filter related work and publication scope

Checked November 19, 2026 against primary author/publisher abstracts and
the official IEEE RAS publication guidance. This is a focused overlap
check, not a systematic literature review or a claim to have reproduced
these methods.

| Work | What the primary source establishes | Implication for our work |
|---|---|---|
| [DynaVINS, Song et al., 2022](https://arxiv.org/abs/2208.11500) | Uses IMU-preintegration pose priors in robust bundle adjustment to reject dynamic features, and addresses temporarily static objects in loop closure. | An IMU prior for rejecting dynamic features is prior art. A vegetation contribution needs evidence beyond that general idea. |
| [DynaSLAM, Bescos et al., 2018](https://bertabescos.github.io/DynaSLAM/) | Adds moving-object detection using multi-view geometry, deep learning, or both, plus background inpainting, to ORB-SLAM2. | Geometry-based dynamic rejection is prior art. We need to evaluate the hard case of distributed, deforming vegetation instead of describing generic rejection as new. |
| [BANSAC, Piedade and Miraldo, ICCV 2023](https://openaccess.thecvf.com/content/ICCV2023/html/Piedade_BANSAC_A_Dynamic_BAyesian_Network_for_Adaptive_SAmple_Consensus_ICCV_2023_paper.html) | Updates individual inlier scores with a dynamic Bayesian network during RANSAC, uses them for weighted sampling and a stopping criterion. | The new implementation uses temporal pixel-residual filtering before RANSAC. It does not implement BANSAC and does not claim a Bayesian probability of being rigid. |

Our candidate contribution is a vegetation-focused evaluation of
persistent-window residuals, with controlled camera/wind factors and a
measured effect on held-out pose accuracy. That is a research hypothesis,
not established novelty. The new synthetic 2x2 is an implementation check;
it cannot substitute for controlled wind recordings and independent
camera-motion evidence. A full related-work search and real-data ablations
are still needed before making a publication claim.

For a mature robotics result, [IEEE Robotics and Automation Letters](https://www.ieee-ras.org/publications/ieee-robotics-and-automation-letters/)
is a possible publication route to discuss. The official page describes
RA-L journal publication and an opportunity for accepted papers to be
presented at eligible RAS conferences. It currently states there are no
conference-specific RA-L submission deadlines and gives a transfer window
within 270 days of acceptance. RA-L is a journal; conference presentation
is a separate eligibility/transfer step. Check the official window again
when a manuscript is ready. This is not a claim that this project is
currently ready to submit, and no team-channel post was made.
