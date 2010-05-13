lx = 130;
ly = 150;
lz = 20;
rx = -130;
ry = 150;
rz = 20;
ldx = 0;
ldy = 0;
ldz = 55;
rdx = 0;
rdy = 0;
rdz = 55;

v = -10;
r = 60;
q = 70;

lz = lz + r;
rz = rz + r;

t = 1000;
t2 = 1200;

max = 5;

smax = 60;

print("aufstehen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;

sleep(t2);

while (max > 0) do
max = max -1;

ldx = smax;
rdx = smax;
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;

sleep(t2);

ldx = -smax;
rdx = -smax;
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;
sleep(t2);

endwhile;

ldx = 0;
rdx = 0;
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;


