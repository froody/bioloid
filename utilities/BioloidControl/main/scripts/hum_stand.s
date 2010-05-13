z = -187.5;
print("Preparing to hold position.\n");
x = pos(2,"x") - 33.5;
t = 1000;
sync
moveto(3,100,0,-60,t);
moveto(4,-100,0,-60,t);
moveto(1,45,0,z, 0,-5, 0,t);
moveto(2,-45,0,z, 0,-5, 0,t);
endsync;
t = 0;
while t < 2000 do
t = t + 200;
sleep(200);
endwhile;
r = collision();
if (r == 0) then
sleep(2000);
print("Holding position.\n");
hold();
else
print("Collision detected. Aborted.\n");
endif;