function T = Denavit_Hartenberg(alpha,a,d,phi)
%     alpha = (alpha)*pi/180;
%     phi = (phi)*pi/180;
    T = [cos(phi) -sin(phi) 0 a; 
        sin(phi)*round(cos(alpha)) cos(phi)*round(cos(alpha)) -round(sin(alpha)) -d*round(sin(alpha));
        sin(phi)*round(sin(alpha)) cos(phi)*round(sin(alpha)) round(cos(alpha)) d*round(cos(alpha));
        0 0 0 1];
end