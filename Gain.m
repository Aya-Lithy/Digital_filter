function [g,h1,w1] = Gain(Z,P,ang,f,g_z,g_p)
%% function 
    [num1, den1]=zp2tf(Z,P,1);
    num1 = poly(Z);
    den1 = poly(P);
    [h1,w1]=freqz(num1,den1);
    %%equation
    for i=1:length(Z)
       for j=1:f  
        xp_UNI_CI(j) = 1*cos(ang(j));
        yp_UNI_CI(j) = 1*sin(ang(j));
        len(j)=sqrt((xp_UNI_CI(j)-real(Z(i)))^2+(yp_UNI_CI(j)-imag(Z(i)))^2);
        l(i,j)=len(j);
       end 
    g_z(1,:)=g_z(1,:).*l(i,:);
    end

    for i=1:length(P)
        for j=1:f    
         xp_UNI_CI(j) = 1*cos(ang(j));
         yp_UNI_CI(j) = 1*sin(ang(j));
         des(j)=sqrt((xp_UNI_CI(j)-real(P(i)))^2+(yp_UNI_CI(j)-imag(P(i)))^2);
         d(i,j)=des(j);   
        end
    g_p(1,:)=g_p(1,:).*d(i,:); 
    end
    g(1,:)=g_z(1,:)./g_p(1,:);

end

