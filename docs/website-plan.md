# GrassVIO project website plan

## Purpose

Give project reviewers a durable, easy-to-scan overview of the GrassVIO work, the evidence behind each result, and the current limitations. The site should link to reproducible code and derived figures without hosting raw recordings.

## Page structure

- **Overview:** one paragraph on the problem and the project approach, with a short pipeline diagram and links to the latest results.
- **Wind and vegetation:** show moving-tree and blower-driven grass cases side by side. For each clip, show the original frame, tracked points, and which points pass or fail each check. Keep the temporal and optical-flow checks visually distinct.
- **Motion and SfM:** show the shared ground-frame camera path, sparse reconstruction, and interval residual plots. State which sequence each result uses and whether a comparison is a reference or independent ground truth.
- **Panoramas and routes:** show the offline panorama and the RGB-D route overlay when that overlay is available. Label the 074200 replay separately so it is not mistaken for the offline-panorama result.
- **Reproduction and status:** link to scripts, commands, data requirements, and a short status table for completed, in-progress, and not-yet-validated checks.

## How to present results

Use a consistent result card for each recording: recording ID, sensor inputs, method, checks run, key output, and limitations. Keep the original image and the annotated output together. Use short captions and readable colors that stand out on grass. Never describe ORB-SLAM3 as ground truth; call it a comparison reference.

Only publish derived figures and small demo clips that the team has permission to share. Keep raw bags, private recordings, credentials, and machine-specific paths out of the site.

## Hosting recommendation

Use **GitLab Pages** for the long-lived project site, as discussed by the team. Keep the source code repository as the source of truth and publish a static site from a team-controlled GitLab project. If the code remains on GitHub, mirror the repository to GitLab or have CI publish the built site there.

The site should build from version-controlled Markdown and static assets, with a Pages CI job that publishes only the generated site directory. Pin the build dependencies and keep the build independent of local recording files so anyone with repository access can reproduce it.

## Suggested first release

Start with a small static site containing the overview, the completed shared-world-frame figures, the current wind-check figures, and the reproduction/status page. Add the offline panorama route only after the RGB-D trajectory has been placed and visually checked on that offline panorama. This keeps the site clear about what is already demonstrated and what is still pending.
