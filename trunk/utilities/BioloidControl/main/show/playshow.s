z = -187.5;
print("Preparing ... 4s left.")	; print(newline);
t = 1000;
sync
arml(100,0,-55,t);
armr(-100,0,-55,t);
legl(45,0,z,-4,0,0,t);
legr(-45,0,z,-4,0,0,t);
endsync;
sleep(4000);
print("Holding position.");print(newline);
hold();