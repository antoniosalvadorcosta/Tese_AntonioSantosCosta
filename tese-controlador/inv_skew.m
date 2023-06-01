function x = inv_skew(A)

n = size(A,1);

if n == 3
     
    x = [A(3,2);A(1,3);A(2,1)];  

elseif n== 2
    
    x = A(2,1);
else
    error('erro');
end

