lx = 130;
ly = 150;
lz = 75;
rx = -130;
ry = 150;
rz = 75;
ldx = 0;
ldy = 0;
ldz = 30;
rdx = 0;
rdy = 0;
rdz = 30;

v = -10;
vv= 0;
hh = 20;
h = 0;
r = 65;
q = -70;

t = 1000;
t2 = 1200;

max = 5;

print("aufstehen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz+h, t);
endsync;

rdz = 0;
ldz = 0;
sleep(t2);
print("l1, l3 und r2 anheben.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;

rdy = q;
ldy = q;
sleep(t2);
print("l1, l3 und r2 schwingen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;


rdz = r;
ldz = r;
sleep(t2);
print("l1, l3 und r2 aufsetzen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;




print("Los gehts.");

while (max > 0) do
max = max - 1;

sleep(t2);
print("l2, r1 und r3 schwingen. l1, l3 und r2 stemmen.\n");
sync
ldx = 0;
ldy = 0;
rdx = 0;
rdy = 0;
rdz = 0;
ldz = 0;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t/4);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t/4);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz+h, t/4);
ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = 0;
ldz = 0;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t/2);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t/2);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz+h, t/2);

ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = r;
ldz = r;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t/4);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t/4);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz+h, t/4);

ldx = 0;
rdx = 0;
ldy = 0;
rdy = 0;
rdz = r;
ldz = r;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t);
moveto(5, rx+rdx, 0+rdy, rz+ldz, t);
endsync;


sleep(t2);
print("l2, r1 und r3 stemmen. l1, l3 und r2 schwingen.\n");
sync
ldx = 0;
rdx = 0;
ldy = 0;
rdy = 0;
rdz = r;
ldz = r;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz+h, t);

ldx = 0;
rdx = 0;
ldy = 0;
rdy = 0;
rdz = 0;
ldz = 0;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t/4);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t/4);
moveto(5, rx+rdx, 0+rdy, rz+ldz, t/4);

ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = 0;
ldz = 0;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t/2);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz+h, t/2);
moveto(5, rx+rdx, 0+rdy, rz+ldz, t/2);

ldx = 0;
ldy = q;
rdx = 0;
rdy = q;
rdz = r;
ldz = r;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t/4);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz, t/4);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t/4);
endsync;

endwhile;

ldx = 0;
ldy = 0;
ldz = r;
rdx = 0;
rdy = 0;
rdz = r;
sleep(t2);
print("ausgangsposition.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy+hh, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy+hh, rz+rdz, t);
endsync;
