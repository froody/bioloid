z = -120;
n = -5;
sync
moveto(1,45,0,z, 0, n, 0, t1);
moveto(2,-45,0,z, 0, n, 0, t1);
endsync;
while z > -185 do
z = z-10;
sync
moveto(1,45,0,z, 0, n, 0, t2);
moveto(2,-45,0,z, 0, n, 0, t2);
endsync;
endwhile;