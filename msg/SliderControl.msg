# FUYU Gantry Slider Control 

# 0 (Stop), 1 (Zeroing), 2 (Absolute Move), 3(Relative Move)  
int16 mode

# There are two x slider but act like as one group
# They can not be controlled seperatelly even by PLC signal
# [mode:0-1, [1 (act) or 0 (not)]] 
# [mode:2, [pos,vel,acc], [mm,mm/s,mm/s2]] // -1 to ignore
# [mode:3, [loc,vel,acc], [mm,mm/s,mm/s2]] // -1 to ignore
int32[] x 


# [mode:0-1, [1 (act) or 0 (not)]] 
# [mode:2, [pos,vel,acc], [mm,mm/s,mm/s2]] // -1 to ignore
# [mode:3, [loc,vel,acc], [mm,mm/s,mm/s2]] // -1 to ignore
int32[] y

