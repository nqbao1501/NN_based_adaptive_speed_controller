s = tf('s');

num = [0.1292 43.58 10.22];
den = [1 108.7 585.7 95.92];
Td  = 0.004;   % time delay (seconds)

G = tf(num, den, 'InputDelay', Td);
pidTuner(G);
