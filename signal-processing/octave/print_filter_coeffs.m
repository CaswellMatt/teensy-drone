sf   = 1000; sf2 = sf/2;
n    = 3;
f    = 80;
Rp   = 4;
Rs   = 30;
side = 60;
lowerBound = f - side;
upperBound = f + side;
Wn = [lowerBound, upperBound]/sf2;
filterType = 'stop';

% Transfer function design
#[b, a]=butter(7, 220/sf2);
#[b, a]=ellip(n, Rp, Rs, 160/sf2);
#ellip(n, Rp, Rs, Wn, filterType);
[b2,a2] = butter(n, Wn, filterType);

#disp(b)
#disp(a)

disp(b2);
disp(a2);