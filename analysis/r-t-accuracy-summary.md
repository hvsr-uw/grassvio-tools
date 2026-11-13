r and t accuracy

sim test, 30 pairs on 155419, true motion known. sim numbers are from don's oct 25 check.

| descriptor | result |
|---|---|
| sift | stays on the true motion |
| bfm | close behind sift |
| orb | off by about 1 degree in rotation and up to 1 cm in translation |
| hog | no pose at all |

sensitivity, frame to frame jitter.

| descriptor | jitter |
|---|---|
| sift | about 0.3 mm |
| hog | about 1 mm |
| bfm | about 1 mm |
| orb | 5 to 8 mm |

loop closing, end against start. orb numbers are only listed where i measured them, the rest are our vo only.

| take | orbslam3 | our vo | loop |
|---|---|---|---|
| 080033 | 0.061 m, 13.0 deg | 0.836 m, 67.6 deg | yes |
| 075947 | 0.188 m, 8.8 deg | 0.580 m, 14.8 deg | yes |
| 155419 | 0.14 m, 20.3 deg | 0.108 m, 19.6 deg | yes |
| 085611 | 0.0001 m, 0.01 deg | 0.238 m, 86.3 deg | still take |
| 173807 | - | 4.926 m, 149.6 deg | no |
| 172532 | 0.81 m, 9.3 deg | 6.102 m, 101.0 deg | no |
| 172828 | 0.83 m, 59.0 deg | 5.394 m, 67.3 deg | no |
| 172859 | - | 0.838 m, 29.4 deg | nearly still |
