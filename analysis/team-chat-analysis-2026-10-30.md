team chat, oct 25 to oct 30

this covers the team chat from oct 25 to oct 30. the quotes below are from the chat itself.

oct 25, 9:49 pm, don. he ran the simulated data where the true motion is known and plotted estimate against truth frame by frame, 30 pairs on 155419. sift stays on the true motion. bfm is close. orb jumps by about a degree and up to a centimeter. hog gave no pose on the combined motion. don asked if the fig 4 curve should switch from orb to sift, since sift is much steadier and follows the imu rotation.

oct 25, 9:53 pm, jinming. "ORB is very bad." use log scale or plot another set without orb.

oct 25, 10:04 pm, jinming. "Thanks a lot, Gents! And nice work!" he said he will talk to us tomorrow.

oct 26, 2:03 pm, jinming. thanks don for the writing updates. asks julius and me to write up our parts today.

oct 26, 2:05 pm, julius. "Yes I will just intergrate my stuff in it wont take to long." "Thank you for figures Don"

oct 26, 2:27 pm, me. doing more experiments to improve the final stitch image and researching recent techniques. will update shortly. asked about the midterm deadline, it was not in the calendar.

oct 26, 2:28 pm, julius. deadline is wednesday, the professor posts at piazza.

oct 26, 5:40 pm, julius. "I updated my part"

oct 26, 6:14 pm, jinming. "Thanks Julius!" asks for fig 4 as a vector version rather than raster. if tikz was used, include the tikz code.

oct 26, 7:37 pm, jinming. "Harsh RATHORE Thanks for writing up your portion! Looks nice." uploads new recordings with loops and wind. "Everyone This should help us to further test our scripts."

oct 27, 12:07 am, me. "do we have everything for the midterm submission?"

oct 27, 5:27 am, jinming. "Looks great. Will let you know should I need anything." he said he will reorganize the document tonight into the final submission format.

oct 27, 9:40 am, julius. "I replaced Fig4 with TikZ"

oct 27, 6:37 pm, jinming. he will shorten the report to 3 to 4 pages, copying all existing contents into a new file grassVIO_report.tex. asks don to adjust the simulation figure from 3 column to 2 column like the other figures.

oct 27, 7:22 pm, don. "Done!!"

oct 27, 7:31 pm, jinming. "Thanks Don!"

oct 27, 9:06 pm, jinming. "Hi Everyone Thanks all for your hardwork! I managed to condense to 4 pages" he said he will take a further look tomorrow and try to submit tomorrow night. asks for feedback here or directly in overleaf.

oct 28, 9:03 am, julius. "this version looks pretty completed" / "Thanks"

oct 28, 1:04 pm, me. "Looks Solid"

oct 28, 8:35 pm, jinming. "Just updated and submitted the midterm project report." submitted oct 28 at 9:33 pm, grassVIO_midterm_report.pdf.

oct 28, 8:42 pm, jinming to me. "I noticed that you have downloaded my latest recordings. I was wondering if you could help us comb through all the recordings, and make a table on what features each recording has, such wind, loop, translation, rotation, translation + rotation etc." second message: "With your stitching script, is there a way to impose the camera trajectory on top of the final panorama?"

oct 29, 4:07 pm, me. "Okay I will look into it." also asked whether the team wants to schedule a slot for the presentation.

oct 29, 4:09 pm, julius. "Yes I don't have any schedule conflicts on those slot." "Select whenever you guys comfortable"

oct 29, 6:54 pm, jinming. "Great point. Let's discuss this at our meeting tomorrow." the meeting was fri oct 30.

here is what the numbers say

sim validation, 30 pairs on 155419, true motion known. sift tracks truth. bfm close behind. orb off by about 1 degree in rotation and up to 1 cm in translation. hog returns no pose at all on the combined motion. sensitivity: sift jitters about 0.3 mm frame to frame, orb 5 to 8 mm.

fig 4: the orb curve is out, jinming said orb is very bad. julius replaced fig 4 with a tikz vector version. the tikz fig 4 is an imu pipeline diagram, which is a different figure than the motion curve, so i still need to check what fig 4 actually shows in the submitted pdf.

here is who did what

don: simulated validation with the truth plots, figure reformatting from 3 column to 2 column, writing updates.

julius: imu section, fig 4 tikz version, deadline info.

me: panorama section, tested the new recordings oct 27, agreed to the feature table and the trajectory work.

jinming: put it all together, condensed to 4 pages, submitted, new recordings, all the coordination.
