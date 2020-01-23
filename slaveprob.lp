\* Slave solver *\
Minimize
OBJ: - 0.5 S1 - 0.5 S10 - S11 - S3 - 0.5 S4 - 0.5 S5 - 0.5 S6 - 0.5 S7 - S8
 - S9
Subject To
_C1: 6 S0 + 10 S1 + 7 S10 + 15 S11 + 5 S2 + 18 S3 + 9 S4 + 12 S5 + 8 S6
 + 11 S7 + 14 S8 + 17 S9 <= 20
Bounds
0 <= S0
0 <= S1
0 <= S10
0 <= S11
0 <= S2
0 <= S3
0 <= S4
0 <= S5
0 <= S6
0 <= S7
0 <= S8
0 <= S9
Generals
S0
S1
S10
S11
S2
S3
S4
S5
S6
S7
S8
S9
End
