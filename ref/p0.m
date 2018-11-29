function y = p0(nue,sigma,x)

vf = 2/(sigma^2);

if x < nue || x > (nue+sigma)
  y = 0;
else
  y = vf*(x-nue);
end
endfunction 
