lx = 130;
ly = 150;
lz = 30;
rx = -130;
ry = 150;
rz = 30;
ldx = 0;
ldy = 0;
ldz = 55;
rdx = 0;
rdy = 0;
rdz = 55;

v = -10;
r = 0;
q = 70;
print(pos(1, "z"));
if (pos(1, "z") < 100) then
r = 70;
endif;

lz = lz + r;
rz = rz + r;

if (paramcount > 0) then
t = param0;
else
t = 1000;
endif;
t2 = t * 1.2;

max = 7;

print("aufstehen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;

sleep(t);