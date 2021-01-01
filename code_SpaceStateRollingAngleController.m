
%% Created November 2020, Last revision: 26-December-2020
%% @author: Izaias A S Jr.
%% email:izaiasjunior747@gmail.com
%%
%The program presented below can be used to calculate the gains of the states Kcl = [k1 k2..Ka]
%for any control system. Therefore, just run the code and provide the linear model of the 
%system to be controlled and the performance specifications of the system's step response. 
%The desired poles also need to be inserted.






%-------------BEGIN CODE--------------------------------------------------
clc
clear all

'TESTES DE MANIPULA�AO DE DADOS'

nSist=input('DIGITE  A ORDEM DO NUMERADOR: ');
fprintf('DIGITE O TERMOS DO NUMERADOR EM SEGUENCIA \n');
fprintf('COME�E PELO TERMO DE MAIOR GRAU \n');
nSist=nSist+1;

for i=1:nSist
   aux1=input('TERMO:');
   Vnum(i)= aux1;
   
end

dSist=input('DIGITE  A ORDEM DO DENOMINADOR: ');
fprintf('DIGITE O TERMOS DO DENOMINADOR EM SEGUENCIA \n');
fprintf('COME�E PELO TERMO DE MAIOR GRAU \n');
nPolos=dSist;
dSist=dSist+1;
for i=1:dSist
   aux1=input('TERMO:');
   Vden(i)= aux1;
   
end
%REPRESENTA�AO DO SITEMA EM ESPA�O DE ESTADOS(A,B,C,D)---------------------
[A,B,C,D]=tf2ss(Vnum,Vden);
GMA=ss(A,B,C,D)
g=tf([Vnum],[Vden])


os=input ('Digite %UP desejada: '); % percentual desejada.
ts=input('Digite o tempo de acomoda��o desejado: '); % Ts desejado
z=(-log(os/100))/(sqrt(pi^2+log(os/100)^2));   %Calcula o fator de amortecimento
wn=4/(z*ts); 
fprintf('ZETA=%.5f \n',z);
fprintf('WN=%.5f \n',wn);
fprintf('PARTE REAL=%.5f \n',z*wn);
fprintf('PARTE IMAGINARIA=%.5f \n',wn*sqrt(1-z^2));
fprintf('Insira os polos desejados! \n')

    for i=1:nPolos+1
        fprintf('Digite o polo %d:',i )
        polos(i)=input('');
    end
    
F=zeros(nPolos+1);
polos = sort(polos,'descend');

j=1;
for i=1:nPolos+1
    F(i,j)=real(polos(i));
    if imag(polos(i))>0
        F(i,j+1)=imag(polos(i));
    elseif imag(polos(i))<0
        F(i,j-1)=imag(polos(i));
    else
        F(i,j)=real(polos(i)); 
    end
j=j+1;
i=i+1;
end
Aext1=[A;C];
Aext=[Aext1 zeros(nPolos+1,1)];
Bext=[B;0];
K=ones(1,length(Aext)); 
T=lyap(Aext,-F,-Bext*K);
Kcl=K*inv(T);    %Obten�ao dos ganhos de ajuste de estado K e ganho do integrador Kcl=[K Ka].
 
%FECHAMENTO DA MALHA DE CONTROLE COM OS GANHOS ENCONTRADOS-----------------
Acl= A-B*[ Kcl(1,1:length(Kcl)-1)]; %Determina�ao da matriz de estados A de malha fechada com os ganhos K=[K1 k2 ....];
GMF1=ss(Acl, B,C,D);   %fun�ao em espa�o de estados da malha fechada sem o integrador (1� malha).
nInt=Kcl(length(Kcl));
dInt=[1 0];
int=tf([nInt],[dInt]);   %Ka=ultima posi�ao do vetor Kcl.
GMF2=feedback(int*GMF1,1,-1); %Fechamento damalha como o integrador (2� malha)
stepinfo(GMF2)
Kcl  

'ANALISE FREQUENCIAL ===================================================='
%the Bode diagram of the expression GW provides a relationship between
%input W and output Y.
syms s;
I=[1 0;0 1];
M=inv(s*I-Acl);
GW=C*M*B;
GW=simplify(GW)
