lx = 130;
ly = 150;
lz = 80;
rx = -130;
ry = 150;
rz = 80;
ldx = 0;
ldy = 0;
ldz = 30;
rdx = 0;
rdy = 0;
rdz = 30;

v = -10;
r = 50;
q = 70;

t = 500;
t2 = 800;

if (paramcount > 0) then
max = param0;
else
max = 3;
endif;

print("aufstehen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;

rdz = 0;
ldz = 0;
sleep(t2);
print("l1, l3 und r2 anheben.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;

rdy = q;
ldy = q;
sleep(t2);
print("l1, l3 und r2 schwingen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;


rdz = r;
ldz = r;
sleep(t2);
print("l1, l3 und r2 aufsetzen.\n");
sync
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;

while (max > 0) do
max = max - 1;

sleep(t2);
print("l2, r1 und r3 anheben. l1, l3 und r2 aufsetzen.\n");
sync
ldx = 0;
ldy = 0;
rdx = 0;
rdy = 0;
rdz = 0;
ldz = 0;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);

ldx = 0;
ldy = q;
rdx = 0;
rdy = q;
rdz = r;
ldz = r;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
endsync;


sleep(t2);
print("l2, r1 und r3 schwingen. l1, l3 und r2 stemmen.\n");
sync
ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = 0;
ldz = 0;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);

ldx = 0;
rdx = 0;
ldy = 0;
rdy = 0;
rdz = r;
ldz = r;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+ldz, t);
endsync;


sleep(t2);
print("l2, r1 und r3 aufsetzen. l1, l3 und r2 anheben.\n");
sync
ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = r;
ldz = r;
moveto(2, lx+ldx, 0+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);

ldx = 0;
rdx = 0;
ldy = 0;
rdy = 0;
rdz = 0;
ldz = 0;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
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
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);

ldx = 0;
rdx = 0;
ldy = q;
rdy = q;
rdz = 0;
ldz = 0;
moveto(1, lx+ldx, ly+ldy, lz+ldz+v, t);
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(5, rx+rdx, 0+rdy, rz+ldz, t);
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
moveto(3, lx+ldx, -ly+ldy, lz+ldz, t);
moveto(4, rx+rdx, ly+rdy, rz+rdz+v,  t);
moveto(5, rx+rdx, 0+rdy, rz+rdz, t);
moveto(6, rx+rdx, -ly+rdy, rz+rdz, t);
endsync;
