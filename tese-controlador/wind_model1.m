function Vw  = wind_model1(Wv,u,v,R,Parameter)

P = Parameter;

% Wind speed felt on the body
Vw = R*Wv*10;