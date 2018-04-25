clear

px = [ 149, 453, 734];
py = [-90, 0, 90];

# y = m*x + b
m = 0.308;                
b = -136.955;

dx = 0.1;
x = [0 : dx : 1023];
y = m*x + b;

figure();
plot(x, y, '-b', 'markersize' , 30);
grid; 

hold on;
plot(px, py, '*r', 'markersize' , 30); 

