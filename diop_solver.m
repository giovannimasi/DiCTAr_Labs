function [R1,S1, Ms] = diop_solver(Adioph, Bdioph, Am, degR1, n_add, extra_eq_matrix)

% funzione per risolvere le eq del cazz 

% Si può chiedere in output anche solo R1 e S1; si possono fornire in input
% semplicemente i primi 4 termini se l'esercizio non ha condizioni extra

% il codice si commenta da solo

% NB se c'è solo un'eq extra extra_eq_matrix è semplicemente un vettore,
% altrimenti diventa una matrice che ha su ogni riga una equazione
% aggiuntiva


Am = Am(:);

if nargin < 5
    n_add = 0;
    Ms = zeros(length(Am));
    Gamma = Am;
else
    Ms = zeros(length(Am)+ n_add);
    Gamma = [Am ; zeros(length(n_add),1)];
end


for ii=1:length(Am)-length(Adioph)+1 % nota: siccome l'ultima riga non devo toccarla, allora non devo fare length(Gamma) - ... ma length(Am)
    Ms(ii:ii+length(Adioph)-1,ii) = Adioph;
end

for jj=1:length(Gamma)-ii % nota: in questo caso invece siccome arrivo fino all'ultima colonna userò length(Gamma) al posto di length(Am)
    Ms(jj+1:jj+length(Bdioph),ii+jj) = Bdioph;
end

if n_add>0

    for kk=1:length(n_add)
        Ms(end-kk+1,:) = extra_eq_matrix(kk,:);
    end

end

theta = Ms \ Gamma;

R1 = theta(1:degR1+1)'; % nota: trasposto perché il comando tf vuole vettori riga
S1 = theta(degR1+2:end)';

end