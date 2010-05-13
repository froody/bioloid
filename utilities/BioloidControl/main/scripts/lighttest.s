dist = 0;
p = 100;
max = 100;
t = 0;
while (t < max) do
t = t + 1;
dist = sensor("LIGHTCENTER");
if (dist > 200) then
t = 1000;
endif;
print(t);
print(" dist: ");
print(dist);
print("\n");
sleep(p);
endwhile;