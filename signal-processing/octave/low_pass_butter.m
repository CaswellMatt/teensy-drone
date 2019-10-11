 sf = 1000; sf2 = sf/2;
 data=[[1;zeros(sf-1,1)],sinetone(100,sf,1,1),sinetone(150,sf,1,1),sinetone(200,sf,1,1)];
 [b,a]=butter ( 6, 150 / sf2 );
 filtered = filter(b,a,data);
  
 [h, w]   = freqz(b, a, length(filtered));
 
 
 disp(b)
 disp(a)
 
 plotCount = 5
 
 clf
 subplot ( plotCount, 1, 1)
 plot(filtered(:,1),";Impulse response;")
 subplot ( plotCount, 1, 2 )
 plot(filtered(:,2),";1 response;")
 subplot ( plotCount, 1, 3 )
 plot(filtered(:,3),";2 response;")
 subplot ( plotCount, 1, 4 )
 plot(filtered(:,4),";3 response;")
 subplot ( plotCount, 1, 5 )
 plot(w/pi,20*log10(abs(h)),";freq response;")
 
#pkg install -forge signal
#pkg load signal