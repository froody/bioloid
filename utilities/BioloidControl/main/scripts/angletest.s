servo = 14;
steps = 15;
stepsize = 5;
pause = 100;
a = get(servo);
print("Current Angle of Servo ");print(servo);print(": ");
print(a);
print("\n");
t = 0;
while (t < steps) do
print("New Target Angle: ");print(a + t*stepsize);print("\n");
set(servo, a + t*stepsize, pause);

t = t + 1;
endwhile;
t = 0;
while (t < steps) do
print("New Target Angle: ");print(a + t*stepsize);print("\n");
set(servo, a + (steps-t)*stepsize, pause);

t = t + 1;
endwhile;