# FUYU Gantry Slider Control 

# 0 (Stop), 1 (Zeroing), 2 (Absolute Move), 3(Relative Move)  
int16 mode

# There are two x slider but act like as one group
# They can not be controlled seperatelly even by PLC signal
# [mode:0-1, [1 (act) or 0 (not)]] 
# mode 2,3 : Unit [mm,mm/s,mm/s2]
# mode 2 : pos(0) is valid, vel and acc <=0 means keeps default
# mode 3 : pos vel and acc <=0 means keeps default
# [mode:2, [pos,vel,acc], ]
# [mode:3, [dis,vel,acc], [mm,mm/s,mm/s2]] 
int32[] x 

# same as x
int32[] y

