
%% Created August 2020, Last revision: 28-December-2020
%% @author: Izaias A S Jr.
%% email:izaiasjunior747@gmail.com
%%
%This program was used for the development of the controller project by 
% a polynomial approach for an airplane angle control system. The Model g(s) used throughout this project, 
% is available at: "Modern control systems" Richard C. Dorf, eighth edition, page 176.






%------------- BEGIN CODE --------------

clear all
clc
%Pesired poles according to the project specifications.
a=[1 4+3.2528i]; 
b=[1 4-3.2528i];
c=[1 40]; 


ds=conv(conv(a,b),c);
r=roots(ds)'
coef=poly(r')

%Construction of matrix E and declaration of the system.
E=[9 0 1 0;
    4 9 0 1;
    1 4 0 0;
    0 1 0 0];

g=tf([1],[1 4 9])
disp('==============MATRIZ E(SISTEMA/PLANTA) e VETOR D==============')
E;
g;
D=conv(conv(a,b),c);
D=fliplr(D);
disp(fliplr(D));
%Controller calculation.
disp('==============CONTROLADOR ==============')
M=inv(E)*D'
C=tf([M(4) M(3)],[M(2) M(1)]);

%Performance analysis of control topologies.
%controller in the direct branch.
Gmf1=tf(feedback((C*g),1));

disp('==============RESULTADOS CONTROLADOR NO RAMO DIRETO  ==============')
stepinfo(Gmf1)
zpk(g);
zpk(C);
zero(Gmf1);

%controller in realimneta�ao
disp('==============RESULTADOS CONTROLADOR NA REALIMENTA�AO  ==============')
Gmf2=tf(feedback(g,C));
stepinfo(Gmf2)
fprintf('zero c(s)=%.4f \n',zero(C))

fprintf('polo c(s)=%.4f \n',pole(C))
zpk(g);
zpk(C);
zero(Gmf2);
comp=1/dcgain(Gmf2)
step((1/dcgain(Gmf2))*Gmf2)
hold on
step(Gmf2)
