dist = 0;
p = 100;
max = 100;
t = 0;
while (t < max) do
t = t + 1;
dist = sensor("DISTCENTER");
if (dist > 200) then
t = 1000;
endif;
print(t);
print(" left: ");
print(sensor("DISTLEFT"));
print(" center: ");
print(dist);
print(" right: ");
print(sensor("DISTRIGHT"));
print("\n");
sleep(p);
endwhile;