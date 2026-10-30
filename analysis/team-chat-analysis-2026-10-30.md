# CS 566 team chat analysis: Oct 25-30, 2026

Source: Teams "CS 566 Project" channel screenshots shared Oct 30. This picks up where
meeting-analysis-2026-10-23.md left off. All quotes are verbatim from the chat.

## Timeline

**Sun Oct 25, 9:49 PM — Don Lee.** Validation on simulated data, where true motion is
known, plotted estimate-vs-truth frame by frame (figure: "Simulated combined motion,
20260926_155419: estimate vs known motion (30 pairs)", rotation x/y/z and translation
x/y/z panels). Findings: SIFT stays on the true motion. BFM is close. ORB jumps by
about a degree and up to a centimeter. HOG returned no pose on this combined motion.
Don asks: the Fig. 4 curve is still ORB; SIFT is much steadier and follows the IMU
rotation — should he switch that curve to SIFT?

**Sun Oct 25, 9:53 PM — Jinming Xu.** "ORB is very bad." To see how the other three
perform, use log scale or plot another set without ORB.

**Sun Oct 25, 10:04 PM — Jinming Xu.** "Thanks a lot, Gents! And nice work!"
"I'll talk to you all tomorrow." "Good night!"

**Mon Oct 26, 2:03 PM — Jinming Xu.** Thanks Don for updating the writing again. Asks
Julius and Harsh to write up their portions today.

**Mon Oct 26, 2:05 PM — Julius Zihao Wu.** "Yes I will just intergrate my stuff in it
wont take to long." "Thank you for figures Don"

**Mon Oct 26, 2:27 PM — Harsh.** Doing more experiments to improve the final stitch
image and researching recent techniques; will update shortly. Asks the midterm
deadline (not in the calendar).

**Mon Oct 26, 2:28 PM — Julius.** Deadline is Wednesday; the professor posts at Piazza.

**Mon Oct 26, 5:40 PM — Julius.** "I updated my part"

**Mon Oct 26, 6:14 PM — Jinming Xu.** "Thanks Julius!" Asks for Fig. 4 as a vector
version rather than raster; if TikZ was used, include the TikZ code.

**Mon Oct 26, 7:37 PM — Jinming Xu.** "Harsh RATHORE Thanks for writing up your
portion! Looks nice." Uploads new recordings with loops and wind (Box link
uwmadison.box.com/s/xeb3f1gzlfth55a3i59iqn6x1fmnjg11). "Everyone This should help
us to further test our scripts."

**Tue Oct 27, 12:07 AM — Harsh.** "do we have everything for the midterm submission?"

**Tue Oct 27, 5:27 AM — Jinming Xu.** "Looks great. Will let you know should I need
anything." Will reorganize the document tonight into the final submission format.

**Tue Oct 27, 9:40 AM — Julius.** "I replaced Fig4 with TikZ" (figure: IMU filter
pipeline diagram).

**Tue Oct 27, 6:37 PM — Jinming Xu.** Will shorten the report to 3-4 pages, copying
all existing contents into a new file grassVIO_report.tex. Asks Don to adjust the
simulation figure from 3-column to 2-column like the other figures.

**Tue Oct 27, 7:22 PM — Don Lee.** "Done!!"

**Tue Oct 27, 7:31 PM — Jinming Xu.** "Thanks Don!"

**Tue Oct 27, 9:06 PM — Jinming Xu.** "Hi Everyone Thanks all for your hardwork! I
managed to condense to 4 pages" (Box link uwmadison.box.com/s/c80yakdnkjvl1e4hs5xmc688crzmxh0i).
Will take a further look tomorrow and try to submit tomorrow night. Asks for feedback
here or directly in Overleaf. "Again, many thanks to you all!"

**Wed Oct 28, 9:03 AM — Julius.** "this version looks pretty completed" / "Thanks"

**Wed Oct 28, 1:04 PM — Harsh.** "Looks Solid"

**Wed Oct 28, 8:35 PM — Jinming Xu.** "Just updated and submitted the midterm project
report." (Box link uwmadison.box.com/s/0nsitxjj9eft10883zt65kq0i7mv5b9). Submission
screenshot: Submitted Oct 28 at 9:33pm, GrassVIO_midterm_report.pdf.

**Wed Oct 28, 8:42 PM — Jinming Xu (to Harsh).** "I noticed that you have downloaded
my latest recordings. I was wondering if you could help us comb through all the
recordings, and make a table on what features each recording has, such wind, loop,
translation, rotation, translation + rotation etc." Second message: "With your
stitching script, is there a way to impose the camera trajectory on top of the final
panorama?"

**Thu Oct 29, 4:07 PM — Harsh.** "Okay I will look into it." Also asks whether the
team wants to schedule a slot for the presentation.

**Thu Oct 29, 4:09 PM — Julius.** "Yes I don't have any schedule conflicts on those
slot." "Select whenever you guys comfortable"

**Thu Oct 29, 6:54 PM — Jinming Xu.** "Great point. Let's discuss this at our meeting
tomorrow." (meeting Fri Oct 30)

## Technical findings new since Oct 23

- Don's simulated validation (true motion known, 30 pairs on 20260926_155419): SIFT
  tracks truth, BFM close behind, ORB deviates by about 1 degree in rotation and up
  to 1 cm in translation, HOG returns no pose at all on the combined motion. This
  lines up with Don's Oct 25 sensitivity study (SIFT jitters ~0.3 mm frame to frame,
  ORB 5-8 mm).
- Fig. 4 history: ORB curve -> Jinming rules ORB "very bad", wants log scale or an
  ORB-free plot -> Don asks about switching the curve to SIFT -> Julius replaces
  Fig. 4 with a TikZ vector version. Note: the TikZ Fig. 4 is an IMU pipeline block
  diagram, which reads like a different figure than the motion curve; worth one look
  at the submitted PDF to confirm what Fig. 4 actually shows.
- Midterm document: 15 pages condensed to 4 pages in a new file grassVIO_report.tex
  (Jinming: all existing contents copied over). Submitted Oct 28, 9:33 PM as
  GrassVIO_midterm_report.pdf.

## Decisions made

- Midterm report submitted Oct 28, 9:33 PM by Jinming. Milestone closed.
- Presentation slot: not scheduled yet; discussion deferred to the Oct 30 meeting.
- Harsh accepted Jinming's two asks on Oct 29: (1) the recording feature table,
  (2) looking into the trajectory overlay.

## Open items

1. Recording feature table — Harsh's, agreed Oct 29. In progress (see
   recording-feature-table.md).
2. Camera trajectory overlaid on the panorama — Jinming's question to Harsh.
   Prototype in progress (see trajectory-overlay.md).
3. Presentation slot scheduling — pending the Oct 30 meeting.
4. Team meeting Oct 30 (today).
5. Harsh's panorama section in the 4-page condensation: the placed section ran
   about 3 pages with 3 figures in the 15-page version, so it was almost certainly
   cut down hard. Worth checking the submitted PDF to see what survived, before the
   final report build reuses any of it.
6. Tree-placement resume on take 173807 (Oct 27 test left it at 4,400/19,110 pairs,
   checkpoint saved) — Harsh's call; now lower priority behind items 1-2.
7. Rec letter heads-up to Jinming — Harsh's own send. The "after the midterm"
   trigger has now fired, so this is unblocked.

## Per-person contributions, Oct 25-30

- Don Lee: simulated-data validation with frame-by-frame truth plots, figure
  reformatting (3-column to 2-column), writing updates.
- Julius Zihao Wu: IMU section integration, Fig. 4 TikZ vector replacement,
  deadline info.
- Harsh: panorama section (placed Oct 26, praised by Jinming), downloaded and
  tested the new recordings Oct 27 (stitch + wind + loop analysis), agreed to the
  feature table and trajectory work.
- Jinming Xu: assembly, 4-page condensation, submission, new loop/wind
  recordings, all coordination.
