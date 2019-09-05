 sf = 2000; sf2 = sf/2;
 data=[[1;zeros(sf-1,1)],sinetone(70,sf,1,1),sinetone(150,sf,1,1),sinetone(200,sf,1,1)];
 order=1;
 [b, a]=ellip (5, 1, 90, [.1 .2]);
 filtered = filter(b,a,data);
 
 disp(b)
 disp(a)
 
 clf
 subplot ( columns ( filtered ), 1, 1)
 plot(filtered(:,1),";Impulse response;")
 subplot ( columns ( filtered ), 1, 2 )
 plot(filtered(:,2),";25Hz response;")
 subplot ( columns ( filtered ), 1, 3 )
 plot(filtered(:,3),";50Hz response;")
 subplot ( columns ( filtered ), 1, 4 )
 plot(filtered(:,4),";100Hz response;")