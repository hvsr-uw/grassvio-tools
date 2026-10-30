# Camera trajectory on the panorama — draft note

Jinming's question (Oct 28): with the stitching script, is there a way to impose the camera trajectory on top of the final panorama?

Yes. The stitching script already computes one homography per frame to place it on the panorama, so mapping each frame's centre through its own transform gives the camera's path in panorama pixels. The new script draws that path as a line with start and end markers. It lives in a new file, trajectory_overlay.py, next to stitch_map.py, and it reuses the same matching and warping code, so the overlay lines up with the map exactly.

Demo on the September take 102631c: 25 frames, 0 skipped pairs, the same 672x507 map the plain script makes. The path is small on this take because the camera barely moves (about 47 pixels of travel over the whole take), so the demo mostly shows the start and end markers sitting close together.

Demo on the October take 20261007_080033: 147 frames, 1 skipped pair, median 2251 inliers, the same 2204x1891 map the plain script makes. Here the camera really travels, so the magenta path loops visibly across the map from the green start marker to the red end marker, which land near each other because the camera came back close to where it started. The far end of the map shows the known chain stretch, and the path stretches with it — the overlay inherits drift exactly where the map does, which is the honest limit stated below.

Honest limits. The path is only as good as the chain: where chained homographies drift, the drawn path drifts with them, in exactly the same way the map does. Tree placement also gives one transform per frame, so the same drawing step works there. On takes where the chain blows up, like 173807 with its 57,000-pixel canvas, the trajectory numbers are meaningless and the script should not be trusted there.
