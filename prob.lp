\* 1D cutting stock *\
Minimize
OBJ: Pat0 + Pat1 + Pat10 + Pat11 + Pat13 + Pat14 + Pat15 + Pat16 + Pat17
 + Pat18 + Pat19 + Pat2 + Pat20 + Pat21 + Pat22 + Pat23 + Pat24 + Pat25
 + Pat26 + Pat3 + Pat4 + Pat5 + Pat6 + Pat7 + Pat8 + Pat9
Subject To
C0: Pat0 + 3 Pat14 + Pat15 + Pat22 >= 65
C1: Pat1 + 2 Pat18 >= 29
C10: Pat10 + 2 Pat15 + 2 Pat23 + Pat26 >= 58
C11: Pat11 + Pat24 >= 22
C2: 4 Pat13 + Pat2 + Pat23 + Pat24 + Pat25 + Pat26 >= 70
C3: Pat3 >= 84
C4: 2 Pat17 + Pat20 + Pat4 >= 46
C5: Pat21 + Pat5 >= 18
C6: 2 Pat16 + Pat19 + Pat21 + Pat26 + Pat6 >= 68
C7: Pat19 + Pat20 + Pat7 >= 87
C8: Pat22 + Pat25 + Pat8 >= 80
C9: Pat9 >= 59
Bounds
0 <= Pat0
0 <= Pat1
0 <= Pat10
0 <= Pat11
0 <= Pat13
0 <= Pat14
0 <= Pat15
0 <= Pat16
0 <= Pat17
0 <= Pat18
0 <= Pat19
0 <= Pat2
0 <= Pat20
0 <= Pat21
0 <= Pat22
0 <= Pat23
0 <= Pat24
0 <= Pat25
0 <= Pat26
0 <= Pat3
0 <= Pat4
0 <= Pat5
0 <= Pat6
0 <= Pat7
0 <= Pat8
0 <= Pat9
Generals
Pat0
Pat1
Pat10
Pat11
Pat13
Pat14
Pat15
Pat16
Pat17
Pat18
Pat19
Pat2
Pat20
Pat21
Pat22
Pat23
Pat24
Pat25
Pat26
Pat3
Pat4
Pat5
Pat6
Pat7
Pat8
Pat9
End
