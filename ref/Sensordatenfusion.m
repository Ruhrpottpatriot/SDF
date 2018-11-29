clear all;
Aufgabe = 5.4; %Auswahl der Aufgabe: 4.1, 4.3, 4.6, 5.4

if Aufgabe == 4.1
  %Aufgabe 4.1
  
  %Variablen deklarieren
  iter = 20;                % Anzahl der Iterationen
  sigma = 0.01;             % Wert << 1
  nue = 0.60;               % 0 < nue < 1-sigma
  sw = 0.001;               % Schrittweite von x
  x = [0:sw:1];             % diskretes Array
  p = zeros(1,1001);        % p0 
  T = zeros(iter+1,1001);   % time evolution
  
  % Berechnung der Startwerte p0 und T1
  for i=1:1001
    p(1,i) = p0(nue,sigma,x(1,i));
    x1 = 0.5*x(1,i);                
    x2 = 1-(0.5*x(1,i));
    T(1,i) = 0.5*((p0(nue,sigma,x1))+(p0(nue,sigma,x2)));
  end
  
  % Plot Startgraph p0
  figure(1)
  hold on
  plot(x,p,"r")
  
  % Aproximation von Tn
  for k=1:iter
    for i=1:1001
      x1 = round(((x(1,i)/2)/sw)+1);
      x2 = round(((1-(x(1,i)/2))/sw)+1);
      T(k+1,i) = 0.5*(T(k,x1)+T(k,x2));
    end
    figure(k+1)
    plot(x,T(k,:),"b")
  end
  
  %Bereinigung der Ausgabe der Variablen  
  clear i iter k x x1 x2 sw sigma nue;
end

if Aufgabe == 4.3
  %Aufgabe 4.3
  
  % Variablen deklarieren
  k = pi*(400/3);       %tmax T[q,v] = pi*(400/3)
  l = 1;                %Schrittweite
  t = [0:l:k];          %diskretes Array für Zeitverlauf
  q = 9;                %q nach Vorgabe
  v = 300;              %v nach Vorgabe
  w = q/(2*v);          %w berechnet nach Vorgabe
  A = (v^2)/q;          %A berechnet nach Vorgabe
  n = (k/l)+1;          %Berechnung der Arraygröße
  r = zeros(2,n);       %Trajektorie r(t)
  rp = zeros(2,n);      %Geschwindigkeit r'(t)
  rpp = zeros(2,n);     %Beschleunigung r''(t)
  tv = zeros(2,n);      %Tangentialbeschleunigung tb(t)
  nv = zeros(2,n);      %Normalbeschleunigung nb(t)

  %Berechnung von r(t)
  for i=1:n
    r(1,i) = A*sin(w*t(1,i));       %x(t) der Trajektorie 
    r(2,i) = A*sin(2*w*t(1,i));     %y(t) der Trajektorie
    %Zum live-plotten
    figure(1)
    hold on
    plot(r(1,i),r(2,i))
    title('Trajektorie')
    xlabel('x(t)')
    ylabel('y(t)')
    pause(0.01)
  end

  %Berechnung der Geschwindigkeit(rp), der Beschleunigung(rpp), der Tangentialvektor(tv) und der Normalvektor(nv)
  for i=1:n
    rp(1,i) = (v*cos(w*t(1,i)))/2;              %x(t) der Geschwindigkeit r'(t)
    rp(2,i) = v*cos(2*w*t(1,i));                %y(t) der Geschwindigkeit r'(t)
    rpp(1,i) = ((-1)*q*sin(w*t(1,i)))/4;        %x(t) der Beschleunigung r''(t)
    rpp(2,i) = (-1)*q*sin(2*w*t(1,i));          %y(t) der Beschleunigung r''(t)
    vf = 1/(sqrt((rp(1,i)^2)+(rp(2,i)^2)));     %Berechnung des Vorfaktors "1/|rp(t)|"
    tv(1,i) = vf*rp(1,i);                       %x(t) des Tangentialvektors tv(t)
    tv(2,i) = vf*rp(2,i);                       %y(t) des Tangentialvektors tv(t)
    nv(1,i) = vf*(-1)*rp(2,i);                  %x(t) der Normalbeschleunigung nv(t)
    nv(2,i) = vf*rp(1,i);                       %y(t) der Normalbeschleunigung nv(t)
  end

  %Plot von Tangentialbeschleunigung und Normalbeschleunigung
  figure(3)
  hold on
  subplot(2,1,1)
  plot(tv(1,:),tv(2,:))
  title('Tangentislvektor tv(t)')
  xlabel('x(t)')
  ylabel('y(t)')
  subplot(2,1,2)
  plot(nv(1,:),nv(2,:))
  title('Normalvektor nv(t)')
  xlabel('x(t)')
  ylabel('y(t)')


  %Plot der Werte |r'(t)|, |r''(t)|, r''(t)*tb(t) und r''(t)*nb(t) gegen t
  figure(2)
  hold on
  subplot(2,2,1)
  plot(t,(sqrt(sum(rp.^2))))
  title('Betrag der Geschwindigkeit')
  xlabel('t')
  ylabel('|rp(t)|')
  subplot(2,2,3)
  plot(t,(sqrt(sum(rpp.^2))))
  title('Betrag der Beschleunigung')
  xlabel('t')
  ylabel('|rpp(t)|')
  subplot(2,2,2)
  plot(t,(sum(rpp.*tv)))
  title('Tangentialbeschleunigung tb(t)')
  xlabel('t')
  ylabel('tb(t)')
  subplot(2,2,4)
  plot(t,(sum(rpp.*nv)))
  title('Normalbeschleunigung nb(t)')
  xlabel('t')
  ylabel('nb(t)')


  %Bereinigung der Ausgabe der Variablen
  clear A i k l q w v n vf;
end

if Aufgabe == 4.6 || Aufgabe == 5.4
  %Aufgabe 4.6
  
  %Variablen deklarieren
  I = eye(2);           % 2x2 Einheitsmatrix 
  O = zeros(2,2);       % 2x2 Nullmatrix
  H = [I,O,O];          % H Matrix
  
  % Trajektorie wie 4.3
  k = pi*(400/3);       %tmax T[q,v] = pi*(400/3)
  l = 5;                %Schrittweite (delta t = 5)
  t = [0:l:k];          %diskretes Array für Zeitverlauf 
  n = (k/l)+1;          %Berechnung der Arraygröße
  r = zeros(2,n);       %Trajektorie r(t)
  rp = zeros(2,n);      %Geschwindigkeit r'(t)
  rpp = zeros(2,n);     %Beschleunigung r''(t)  
  q = 9;                %q nach Vorgabe
  v = 300;              %v nach Vorgabe
  w = q/(2*v);          %w berechnet nach Vorgabe
  A = (v^2)/q;          %A berechnet nach Vorgabe
  
  % Berechnung von r(t)
  for i=1:n
    r(1,i) = A*sin(w*t(1,i));       %x(t) der Trajektorie 
    r(2,i) = A*sin(2*w*t(1,i));     %y(t) der Trajektorie
  end
 
  %Berechnung der Geschwindigkeit(rp), der Beschleunigung(rpp)
  for i=1:n
    rp(1,i) = (v*cos(w*t(1,i)))/2;              %x(t) der Geschwindigkeit r'(t)
    rp(2,i) = v*cos(2*w*t(1,i));                %y(t) der Geschwindigkeit r'(t)
    rpp(1,i) = ((-1)*q*sin(w*t(1,i)))/4;        %x(t) der Beschleunigung r''(t)
    rpp(2,i) = (-1)*q*sin(2*w*t(1,i));          %y(t) der Beschleunigung r''(t)
  end
 
  % Berechnung von xk
  xk = transpose([transpose(r),transpose(rp),transpose(rpp)]);
  norm = normrnd(0,1,2,n).*50;
  zk = H*xk+norm;
  
  % Plot zk
  if Aufgabe == 4.6
    for i=1:n
      figure(1)
      hold on
      plot(r(1,i),r(2,i),".r")          % Wirkliche Werte
      plot(zk(1,i),zk(2,i),"k")         % Verrauchte Messung
      title('Trajektorie')
      xlabel('x(t)')
      ylabel('y(t)')
      pause(0.01)
    end
  end
    
  %Bereinigung der Ausgabe der Variablen    
  clear A H i k l norm q t v w r
  if Aufgabe == 4.6
    clear I O n rp rpp xk
  end
end

if Aufgabe == 5.4
  %Aufgabe 5.4
  
  %Variablen deklarieren            % xk und zk aus 4.6
  Pkk = zeros(6,6,n+1);             % 3d Array von P_{k|k}; Filtering
  xkk = zeros(6,n+1);               % Array von x_{k|k}; Filtering
  Pk1k = zeros(6,6,n);              % 3d Array von P_{k|k-1}; Predictions
  xk1k = zeros(6,n);                % Array von x_{k|k-1}; Predictions
  vmax = max(sqrt(sum(rp.^2)));
  qmax = max(sqrt(sum(rpp.^2)));
  
  % Initiation
  Pkk(:,:,1) = [I.*2500,O,O;O,I.*vmax,O;O,O,I.*qmax];
  xkk(:,1) = [zk(1,1);zk(2,1);0;0;0;0];
  %vf = 1/(sqrt(det(Pkk(:,:,1).*(2*pi))));
  %exponent = ((transpose(xk(:,1)-xkk(:,1)))*(inv(Pkk(:,:,1)))*(xk(:,1)-xkk(:,1)))*(-0.5);
  %px0 = vf*exp(exponent);
  
  % Prediction und Filtering
  for i=1:n
    [xk1k(:,i), Pk1k(:,:,i), xkk(:,i+1), Pkk(:,:,i+1)] = Kalman(qmax, zk(:,i), xkk(:,i), Pkk(:,:,i));
  end
  
  % Plot
  figure(1)
  hold on
  plot(xk(1,:),xk(2,:),"k")           % Werte
  plot(zk(1,:),zk(2,:),".r")          % Verrauschte Messung
  plot(xkk(1,:),xkk(2,:),"xb")        % Filtering
  plot(xk1k(1,:),xk1k(2,:),"*g")      % Prediction
  for i=1:n+1
    [e1, e2, X, Y]=ellipse(Pkk(:,:,i),xkk(:,i));
    plot([xkk(1,i); (xkk(1,i)-e2(1,1))],[xkk(2,i); (xkk(2,i)-e2(2,1))],"b");
    plot([xkk(1,i); (xkk(1,i)+e2(1,1))],[xkk(2,i); (xkk(2,i)+e2(2,1))],"b");
    plot(X(1,:),Y(1,:));
    plot(X(2,:),Y(2,:));
    plot([xkk(1,i); (xkk(1,i)+e1(1,1))],[xkk(2,i); (xkk(2,i)+e1(2,1))],"b");
    plot([xkk(1,i); (xkk(1,i)-e1(1,1))],[xkk(2,i); (xkk(2,i)-e1(2,1))],"b");
  end
  
  %Bereinigung der Ausgabe der Variablen
  clear vmax vf qmax n i exponent O I  
end