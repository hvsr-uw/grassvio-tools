r and t accuracy

sim test, 30 pairs on 155419, true motion known. sim numbers are from don's oct 25 check.

sift tracks the true motion. bfm is close behind. orb is off by about 1 degree in rotation and up to 1 cm in translation. hog gives no pose at all on the combined motion.

sensitivity, frame to frame jitter. sift about 0.3 mm. hog and bfm about 1 mm. orb 5 to 8 mm.

loop closing on the real takes, end against start. orbslam3 rgbd.

080033, 0.061 m and 13.0 degrees. our own vo was 0.836 m and 67.6 degrees.

075947, 0.188 m and 8.8 degrees. our own vo was 0.580 m and 14.8 degrees.

155419, 0.108 m and 19.6 degrees.

173807, 4.926 m and 149.6 degrees. not a loop.

172532, 6.102 m and 101.0 degrees. not a loop.

172828, 5.394 m and 67.3 degrees. not a loop.

172859, 0.838 m and 29.4 degrees on a 1.01 m path. nearly still.
