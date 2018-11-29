function [xk1k, Pk1k, xk1k1, Pk1k1] = Kalman(qmax, zk, xkk, Pkk)

% Variablen deklarieren
I = eye(2);
O = zeros(2,2);
F = [I,I.*5,I.*(12.5);O,I,I.*5;O,O,I.*(exp(-1/12))];
vf = (qmax^2)*(1-exp(-1/6));
D = [O,O,O;O,O,O;O,O,I.*vf];
H = [I,O,O];
R = I.*2500;

% Prediction
xk1k = F*xkk;
Pk1k = (F*Pkk*(transpose(F)))+D;

% Filtering
v = zk-(H*xk1k);
S = (H*Pk1k*(transpose(H)))+R;
W = Pk1k*(transpose(H))*(inv(S));

xk1k1 = xk1k+(W*v); 
Pk1k1 = Pk1k-(W*S*(transpose(W)));
endfunction