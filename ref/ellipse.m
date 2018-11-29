function [e1,e2,X,Y] = ellipse(Pkk,xkk)

H = [eye(2),zeros(2,2),zeros(2,2)];


R = H*Pkk*transpose(H);


[V,D] = eig(R);

e1 = V(:,1).*sqrt(D(1,1));
e2 = V(:,2).*sqrt(D(2,2));

x = [-sqrt((abs(D(1,1)))):(abs(D(1,1))/5000):sqrt(abs(D(1,1)))];
y = zeros(1,length(x));

for i=1:length(x)
  tmp1 = sqrt(D(1,1)^2);
  tmp2 = x(1,i)^2;
  tmp3 = sqrt(D(2,2)^2);
  tmp4 = tmp1*tmp2;
  tmp5 = tmp4*inv(tmp3);
  tmp6 = tmp1-tmp5;
  tmp7 = abs(tmp6);
  y(1,i) = sqrt(tmp7);
end
X = [x+(ones(1,length(x)).*xkk(1,1));x+(ones(1,length(x)).*xkk(1,1))];
Y = [-y+(ones(1,length(x)).*xkk(2,1));y+(ones(1,length(x)).*xkk(2,1))];
endfunction