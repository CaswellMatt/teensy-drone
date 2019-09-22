 sf = 800; sf2 = sf/2;
 data=[[1;zeros(sf-1,1)],sinetone(25,sf,1,1),sinetone(50,sf,1,1),sinetone(100,sf,1,1)];
 [b,a]=butter ( 1, 50 / sf2 );
 filtered = filter(b,a,data);

 clf
 subplot ( columns ( filtered ), 1, 1)
 plot(filtered(:,1),";Impulse response;")
 subplot ( columns ( filtered ), 1, 2 )
 plot(filtered(:,2),";25Hz response;")
 subplot ( columns ( filtered ), 1, 3 )
 plot(filtered(:,3),";50Hz response;")
 subplot ( columns ( filtered ), 1, 4 )
 plot(filtered(:,4),";100Hz response;")
 
#pkg install -forge signal
#pkg load signal