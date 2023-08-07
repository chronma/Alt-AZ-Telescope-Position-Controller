// This file defines the GPIO port addresses and PRU address
// Should work for all BB, but wireless versions and video pins will not let some pru pins be set to PRU mode
// I.E.  Config-pin P9.29 pruout will fail on BBG wireless
/* modify /boot/uEnv.txt example turn off video to free up some P8 pins

#disable_uboot_overlay_emmc=1
disable_uboot_overlay_video=1  (remove # to make active)
#disable_uboot_overlay_audio=1
#disable_uboot_overlay_wireless=1
#disable_uboot_overlay_adc=1
linnux cmd to see pinmux
ls /sys/devices/platform/ocp/ | grep pinmux*
*/


/*
bit     pru0's  pru1's  pru0's      pru1's
        __R30   __R30   __R31       __R31
0       P9_31   P8_45   P9_31       P8_45
1       P9_29   P8_46   P9_29       P8_46
2       P9_30   P8_43   P9_30       P8_43
3       P9_28   P8_44   P9_28       P8_44
4       P9_92   P8_41   P9_92       P8_41
5       P9_27   P8_42   P9_27       P8_42
6       P9_91   P8_39   P9_41B      P8_39
7       P9_25   P8_40   P9_25       P8_40
8               P8_27               P8_27
9               P8_29               P8_29
10              P8_28               P8_28
11              P8_30               P8_30
12              P8_21               P8_21
13              P8_20               P8_20
14      P8_12           P9_16
15      P8_11           P8_15
16                  P9_24/P9_41A    P8_26
*/

// R30 output bits / R31 input bits on pru0 (BBG default PINMUX )
#define P8_45   (1UL<<0) // a4 pulse pruout
#define P8_46   (1UL<<1) // a4 dir  pruout
#define P8_43   (1UL<<2) // a5 pulse pruout
#define P8_44   (1UL<<3) // a5 dir pruout
#define P8_41   (1UL<<4) // a6 pulse pruout
#define P8_42   (1UL<<5) // a6 dir pruout
#define P8_39   (1UL<<6) // a4 limit pruin
#define P8_40   (1UL<<7) // a5 limit pruin
#define P8_27   (1UL<<8) // a6 limit pruin






