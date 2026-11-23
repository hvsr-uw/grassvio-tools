# Plot the transformed geometry exported by export_sfm_world_frame.py.

args <- commandArgs(trailingOnly = TRUE)
if (length(args) != 2L) stop("Usage: Rscript scripts/plot_sfm_world_frame.R OUTPUT_DIR CLIP")
outdir <- args[1]
clip <- args[2]
poses <- read.csv(file.path(outdir, "world-poses.csv"))
ply <- file.path(outdir, "world-points.ply")
header <- match("end_header", readLines(ply, n = 20L))
cloud <- read.table(ply, skip = header, col.names = c("x", "y", "z", "r", "g", "b"))
blue <- "#154e9b"
gray <- "#657787"
orange <- "#bc4b21"
ink <- "#192c3a"
green <- "#6a8654"
grid_color <- "#e5ebed"

png(file.path(outdir, paste0(clip, "-sfm-world-frame.png")),
    width = 1900, height = 1100, res = 155, bg = "white")
par(oma = c(4.0, 1.5, 4.0, 1.5), family = "sans", fg = ink, col.axis = ink, col.lab = ink)
layout(matrix(c(1, 1, 2, 3), nrow = 2), widths = c(1.30, 1))

par(mar = c(2.6, 0.4, 2.6, 0.3))
x_limits <- range(cloud$x) + c(-0.04, 0.04)
y_limits <- range(cloud$y) + c(-0.04, 0.04)
z_limits <- range(c(cloud$z, poses$sfm_z, poses$rgbd_z, 0)) + c(-0.02, 0.08)
plane <- persp(x_limits, y_limits, matrix(0, 2, 2), zlim = z_limits,
               theta = -30, phi = 20, expand = 1, scale = FALSE, r = 50,
               xlab = "World X (m)", ylab = "World Y (m)", zlab = "World Z (m)",
               ticktype = "detailed", nticks = 4, cex.axis = .72, cex.lab = .9,
               col = "#edf2e8", border = "#a7b7a0", box = TRUE)
title("Scene points and camera path", cex.main = 1.02, line = .8)
point_color <- rgb(cloud$r, cloud$g, cloud$b, alpha = 190, maxColorValue = 255)
points(trans3d(cloud$x, cloud$y, cloud$z, plane), pch = 16, col = point_color, cex = .28)
lines(trans3d(poses$rgbd_x, poses$rgbd_y, poses$rgbd_z, plane), lwd = 2, col = gray, lty = 2)
lines(trans3d(poses$sfm_x, poses$sfm_y, poses$sfm_z, plane), lwd = 2.7, col = blue)
points(trans3d(poses$sfm_x, poses$sfm_y, poses$sfm_z, plane), pch = 16, col = blue, cex = .48)
height <- poses$sfm_z[1]
guide <- trans3d(c(0, 0), c(0, 0), c(0, height), plane)
lines(guide, col = orange, lty = 3, lwd = 1.65)
points(guide, pch = c(4, 19), col = orange, cex = 1.15, lwd = 2)
segments(guide$x[2], guide$y[2], guide$x[2] + .025, guide$y[2] - .012, col = orange)
text(guide$x[2] + .025, guide$y[2] - .012, sprintf("Frame 0  |  %.2f m", height),
     pos = 4, offset = .4, col = orange, font = 2, cex = .88, xpd = NA)
text(guide$x[1], guide$y[1], "Ground origin", pos = 1, offset = .65,
     col = orange, cex = .75, xpd = NA)

par(mar = c(3.1, 4.7, 2, .6), mgp = c(2.1, .65, 0), tcl = -.27, las = 1)
plot(NA, xlim = range(c(poses$sfm_x, poses$rgbd_x)) * 100 + c(-1.5, 1.5),
     ylim = range(c(poses$sfm_y, poses$rgbd_y)) * 100 + c(-1.5, 1.5), asp = 1,
     xlab = "World X (cm)", ylab = "World Y (cm)", main = "Camera path from above", cex.main = .94)
grid(col = grid_color, lty = 1)
lines(100 * poses$rgbd_x, 100 * poses$rgbd_y, col = gray, lwd = 1.7, lty = 2)
lines(100 * poses$sfm_x, 100 * poses$sfm_y, col = blue, lwd = 2, type = "o", pch = 16, cex = .55)
points(0, 0, pch = 19, cex = 1.05, col = orange)
text(0, 0, "Start", pos = 4, col = orange, cex = .76)

plot(NA, xlim = range(poses$frame), ylim = c(-.075, z_limits[2]), xlab = "Original RGB frame",
     ylab = "World Z (m)", main = "Camera height above the ground", cex.main = .94)
grid(col = grid_color, lty = 1)
abline(h = 0, col = green, lwd = 1.8)
text(mean(range(poses$frame)), 0, "Ground (Z = 0)", pos = 3, col = green, cex = .78)
lines(poses$frame, poses$rgbd_z, col = gray, lwd = 1.7, lty = 2)
lines(poses$frame, poses$sfm_z, col = blue, lwd = 2, type = "o", pch = 16, cex = .55)
points(0, height, pch = 19, cex = 1.05, col = orange)
legend("bottomleft", inset = c(.025, .19), legend = c("SfM", "ORB-SLAM3 RGB-D"),
       lty = c(1, 2), lwd = 2, col = c(blue, gray), cex = .73,
       horiz = TRUE, bty = "n", x.intersp = .55)

mtext(paste(clip, "| SfM in the ground-aligned world frame"), side = 3, outer = TRUE,
      at = .022, adj = 0, line = 1.6, cex = 1.55, font = 2, col = ink)
mtext("X right  /  Y forward along the ground  /  Z up", side = 3, outer = TRUE,
      at = .022, adj = 0, line = .25, cex = .86, col = gray)
mtext(sprintf("%s camera views and %s SfM landmarks  |  One fixed transform for the scene and both paths",
              nrow(poses), format(nrow(cloud), big.mark = ",")), side = 1, outer = TRUE,
      at = .022, adj = 0, line = 1.1, cex = .79, col = gray)
invisible(dev.off())
