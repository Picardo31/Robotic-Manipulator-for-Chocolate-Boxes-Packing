clc
clear all
close all
x = -255.4;
y = 140;
z = 30;

c = 1;
j = 1;
er = 5;
%Min = 1;
sQ1 = [0,0,0,0];
sQ2 = [0,0,0,0];
sQ3 = [0,0,0,0];
sQ4 = [0,0,0,0];
spx = 0;
spy = 0;
spz = 0;
tic
while(1)
    
    for i = 2:1:4
        sQ1(i) = rad2deg(atan2(y,x));
        %sQ1(i) =randi([0 180]);
        sQ2(i) =randi([0 139]);
        sQ3(i) =randi([-138 -50]);
        sQ4(i) = abs(sQ3(i))-sQ2(i);
    end
    
    
    for i = 1:1:4
        
        phi1 = sQ1(i);
        phi2 = sQ2(i);
        phi3 = sQ3(i);
        phi4 = sQ4(i);
        
        [O_T_1, i1_T_2, i2_T_3, i3_T_4, i4_T_T, O_T_T] = DH_Chobot(phi1,phi2,phi3,phi4);

        spx(i) = O_T_T(1,4);
        spy(i) = O_T_T(2,4);
        spz(i) = O_T_T(3,4);
        
    end

    for i = 1:1:4
        Min(i) = abs(x-spx(i))+abs(y-spy(i))+abs(z-spz(i));
    end
    
    j = 1;
    
    for i = 2:1:4
        if (Min(i)<Min(j))
            j = i;
        end
    end
    
    sQ1(1)=sQ1(j);
    sQ2(1)=sQ2(j);
    sQ3(1)=sQ3(j);
    sQ4(1)=sQ4(j);
    
    
%     erx = (abs(x-spx)/x)*100;
%     ery = (abs(y-spy)/y)*100;
%     erz = (abs(z-spz)/z)*100;

     fprintf("c:%d   a1:%d   a2:%d   a3:%d   a4:%d   x:%0.2f   y:%0.2f   z:%0.2f \n",c,sQ1(1),sQ2(1),sQ3(1),sQ4(1),spx(1),spy(1),spz(1))
    
    
   if ((abs(x-spx(1))<=er) && (abs(y-spy(1))<=er) && (abs(z-spz(1))<=er))
       break;
   end
    if (c>1800)
         break;
    end
    c = c + 1;
end
    
phi1f = sQ1(1);
phi2f = sQ2(1);
phi3f = sQ3(1);
phi4f = sQ4(1);
toc


