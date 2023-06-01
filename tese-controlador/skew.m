function X = skew(x)

n = length(x);

if n == 3
    X = [ 0   -x(3) x(2)
          x(3) 0   -x(1)
         -x(2) x(1) 0 ];

elseif n== 1
    X = [0 -x(1)
        x(1) 0] ;
    
else
    error('erro');
end


