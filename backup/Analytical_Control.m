function [tension,MyControlGamma] = Analytical_Control(N,Nd,gamma_hat,N_final)
% clear all; clc; close all;
% % N = [0 -1 0 1 -2 2;0 1 2 1 0 0; 0 0 0 0 0 0];
% N = [rand(2,21);zeros(1,21)];
% Nd = zeros(size(N));
% gamma_hat = zeros(22,22);
% N_final = ones(3,18);


%%  ----------------------- D1 --------------------------------------------
% C_b_com = [-1 1 0 0 0 0; 0 -1 1 0 0 0;0 0 -1 1 0 0;1 0 0 -1 0 0];
% C_s_com = [-1 0 1 0 0 0;0 -1 0 1 0 0;0 0 0 -1 0 1;0 -1 0 0 1 0];
% pinned_nodes = [1 5 6];
% 
% N_final = N_final(:,2:4);
% % tenseg_plot(N,C_b_com,C_s_com)
% % hold on;
% [Nd,~,~,~,~,~] = tenseg_class_k_convert(Nd,C_b_com,C_s_com,pinned_nodes);
% % [N_final,~,~,~,~,~] = tenseg_class_k_convert(N_final,C_b_com,C_s_com,pinned_nodes);
% [N,C_b,C_s,P,D,node_constraints] = tenseg_class_k_convert(N,C_b_com,C_s_com,pinned_nodes);
% R = eye(size(N,2));
% R = R(:,2:4);
% mb = 1*ones(beta,1);
% ms = [.01;.01];

%%  ----------------------- T1D1 ------------------------------------------
% C_b_com = zeros(10,11);
% C_b_com(1,1) = -1; C_b_com(1,2) = 1;
% C_b_com(2,2) = -1; C_b_com(2,3) = 1;
% C_b_com(3,3) = -1; C_b_com(3,4) = 1;
% C_b_com(4,3) = -1; C_b_com(4,5) = 1;
% C_b_com(5,5) = -1; C_b_com(5,6) = 1;
% C_b_com(6,3) = -1; C_b_com(6,7) = 1;
% C_b_com(7,1) = -1; C_b_com(7,8) = 1;
% C_b_com(8,8) = -1; C_b_com(8,3) = 1;
% C_b_com(9,3) = -1; C_b_com(9,9) = 1;
% C_b_com(10,9) = -1; C_b_com(10,6) = 1;
% 
% C_s_com = zeros(10,11);
% C_s_com(1,1) = -1; C_s_com(1,3) = 1;
% C_s_com(2,2) = -1; C_s_com(2,8) = 1;
% C_s_com(3,3) = -1; C_s_com(3,6) = 1;
% C_s_com(4,5) = -1; C_s_com(4,9) = 1;
% C_s_com(5,1) = -1; C_s_com(5,4) = 1;
% C_s_com(6,1) = -1; C_s_com(6,7) = 1;
% C_s_com(7,10) = -1; C_s_com(7,4) = 1;
% C_s_com(8,11) = -1; C_s_com(8,7) = 1;
% C_s_com(9,4) = -1; C_s_com(9,6) = 1;
% C_s_com(10,7) = -1; C_s_com(10,6) = 1;
% pinned_nodes = [1 10 11];
% 
% N_final = N_final(:,2:9);
% % tenseg_plot(N,C_b_com,C_s_com)
% [Nd,~,~,~,~,~] = tenseg_class_k_convert(Nd,C_b_com,C_s_com,pinned_nodes);
% % [N_final,~,~,~,~,~] = tenseg_class_k_convert(N_final,C_b_com,C_s_com,pinned_nodes);
% [N,C_b,C_s,P,D,node_constraints] = tenseg_class_k_convert(N,C_b_com,C_s_com,pinned_nodes);
% R = eye(size(N,2));
% R = R(:,2:9);
% mb = 0.6*ones(size(C_b,1),1);
% ms = [.01;.01];

%  ----------------------- T2D1 ------------------------------------------
C_b_com = zeros(22,21);
C_b_com(1,1) = -1; C_b_com(1,2) = 1;
C_b_com(2,2) = -1; C_b_com(2,3) = 1;
C_b_com(3,3) = -1; C_b_com(3,4) = 1;
C_b_com(4,3) = -1; C_b_com(4,5) = 1;
C_b_com(5,5) = -1; C_b_com(5,6) = 1;
C_b_com(6,6) = -1; C_b_com(6,7) = 1;
C_b_com(7,6) = -1; C_b_com(7,8) = 1;
C_b_com(8,8) = -1; C_b_com(8,9) = 1;
C_b_com(9,9) = -1; C_b_com(9,10) = 1;
C_b_com(10,9) = -1; C_b_com(10,11) = 1;
C_b_com(11,11) = -1; C_b_com(11,12) = 1;
C_b_com(12,9) = -1; C_b_com(12,13) = 1;
C_b_com(13,6) = -1; C_b_com(13,14) = 1;
C_b_com(14,3) = -1; C_b_com(14,15) = 1;
C_b_com(15,1) = -1; C_b_com(15,16) = 1;
C_b_com(16,16) = -1; C_b_com(16,3) = 1;
C_b_com(17,3) = -1; C_b_com(17,17) = 1;
C_b_com(18,17) = -1; C_b_com(18,6) = 1;
C_b_com(19,6) = -1; C_b_com(19,18) = 1;
C_b_com(20,18) = -1; C_b_com(20,9) = 1;
C_b_com(21,9) = -1; C_b_com(21,19) = 1;
C_b_com(22,19) = -1; C_b_com(22,12) = 1;

C_s_com = zeros(22,21);
C_s_com(1,1) = -1; C_s_com(1,3) = 1;
C_s_com(2,2) = -1; C_s_com(2,16) = 1;
C_s_com(3,3) = -1; C_s_com(3,6) = 1;
C_s_com(4,5) = -1; C_s_com(4,17) = 1;
C_s_com(5,1) = -1; C_s_com(5,4) = 1;
C_s_com(6,1) = -1; C_s_com(6,15) = 1;
C_s_com(7,4) = -1; C_s_com(7,6) = 1;
C_s_com(8,15) = -1; C_s_com(8,6) = 1;
C_s_com(9,6) = -1; C_s_com(9,9) = 1;
C_s_com(10,8) = -1; C_s_com(10,18) = 1;
C_s_com(11,9) = -1; C_s_com(11,12) = 1;
C_s_com(12,11) = -1; C_s_com(12,19) = 1;
C_s_com(13,6) = -1; C_s_com(13,10) = 1;
C_s_com(14,6) = -1; C_s_com(14,13) = 1;
C_s_com(15,10) = -1; C_s_com(15,12) = 1;
C_s_com(16,13) = -1; C_s_com(16,12) = 1;
C_s_com(17,1) = -1; C_s_com(17,7) = 1;
C_s_com(18,1) = -1; C_s_com(18,14) = 1;
C_s_com(19,7) = -1; C_s_com(19,12) = 1;
C_s_com(20,14) = -1; C_s_com(20,12) = 1;
C_s_com(21,20) = -1; C_s_com(21,7) = 1;
C_s_com(22,21) = -1; C_s_com(22,14) = 1;
pinned_nodes = [1 20 21];

N_final = N_final(:,2:19);

% tenseg_plot(N,C_b_com,C_s_com)
[Nd,~,~,~,~,~] = tenseg_class_k_convert(Nd,C_b_com,C_s_com,pinned_nodes);
% [N_final,~,~,~,~,~] = tenseg_class_k_convert(N_final,C_b_com,C_s_com,pinned_nodes);
[N,C_b,C_s,P,D,node_constraints] = tenseg_class_k_convert(N,C_b_com,C_s_com,pinned_nodes);
R = eye(size(N,2));
R = R(:,2:19);
mb = 1*ones(size(C_b,1),1);
ms = [.01;.01];

%%

% tenseg_plot(N,C_b,C_s)
L = eye(3);



% Get basic structure values
n = size(N,2); % number of nodes
beta = size(C_b,1); % number of bars
alpha = size(C_s,1); % number of strings
% Check if we have string point mass nodes
sigma = n-2*beta;
% N_b = N(:,1:2*beta);
% N_s = N(:,2*beta+1:end);


% Convert to separate matrices for dynamics (also calc of consts M,Minv)
C_sb = C_s(:,1:2*beta); % string connections to bars
C_ss = C_s(:,2*beta+1:end); % string connections to string nodes
C_bb = C_b(:,1:2*beta); % non-zero columns of C_b (old C_b matrix)
C_r = 1/2*abs(C_b(:,1:2*beta)); % Bar center of mass 'connectivity'
C_nb = [eye(2*beta), zeros(2*beta,sigma)]; % satisfies R_b = N*C_nb^T*C_r'
C_ns = [zeros(sigma,2*beta), eye(sigma)]; % satisfies R_s = N*C_ns'
C_nbTC_bbT = sparse((C_nb') * (C_bb'));
C_sC_nbTC_bbT = sparse(C_s*C_nbTC_bbT);


%%

m_tot_hat = diag(mb);
m_s_hat = diag(ms);
Jt_hat = 1/12*m_tot_hat;

M = [C_nb'*(C_bb'*Jt_hat*C_bb + C_r'*m_tot_hat*C_r), C_ns'*m_s_hat];
Minv = M^-1;

%%
B = N*C_b';
S = N*C_s';
Bd = Nd*C_b';
len_hat = sqrt(diag(diag(B'*B))); % Diag matrices with bar lengths
lend_hat = sqrt(diag(diag(Bd'*Bd))); % Diag matrices with bar vector derivatives
str_len_hat = sqrt(diag(diag(S'*S)));



%%

W = zeros(3,n);
m_hat = diag(mb);
bC=P'*C_b';
bD=C_b*Minv*P;
bE=P'*Minv*P;
E=eye(3);
bA=-S*gamma_hat*C_s*Minv*P+B*...
    diag(diag(.5*len_hat^(-2)*B'*(S*gamma_hat*C_s-W)*C_b'-1/12*len_hat^(-2)*m_hat*(Bd'*Bd)))...
    *C_b*Minv*P+W*Minv*P;
Lag_Mat=0;
for j=1:size(C_b,1)
    Lag_Mat=Lag_Mat+1/(2*len_hat(j,j)^2)*kron(bC(:,j)',kron(B(:,j),(B(:,j)*bD(j,:))'));
end
Lag_Mat=Lag_Mat-[kron(bE,E(1,:));kron(bE,E(2,:));kron(bE,E(3,:))];

OMEGA_LAGRANGE_Vec=Lag_Mat\[bA(1,:)';bA(2,:)';bA(3,:)'];
OMEGA_LAGRANGE=reshape(OMEGA_LAGRANGE_Vec,3,numel(OMEGA_LAGRANGE_Vec)/3);
%%


Wtot = W + OMEGA_LAGRANGE*P';
WC_nbTC_bbT = sparse(Wtot*C_nbTC_bbT);

% Control input
Ybar = N_final;
Psi = 20*eye(size(R,2));
OmegaCon= 30*eye(size(R,2));

LN = L*N;
MinvR = sparse(Minv*R);
MinvRb = MinvR(1:2*beta,:);
MinvRs = MinvR(2*beta+1:end,:);
RHS = L*Wtot*Minv*R + L*Nd*R*Psi + (L*N*R - Ybar)*OmegaCon;
Caxes = size(L,1); % Number of axes of interest
Cnodes = size(R,2); % Number of nodes of interest

%% Find 'Lambda' and 'tau' such that: lambda = Lambda * gamma + tau

% Initializing Lambda and tau
Lambda = zeros(beta,alpha);
tau = zeros(beta,1);

% Rows of Lambda and components of tau

for i = 1:beta
    Lambda(i,:) = (1/(2*len_hat(i,i)^2))* B(:,i)' * S * ...
        diag(C_sC_nbTC_bbT(:,i));
    
    tau(i) = -Jt_hat(i,i)/(len_hat(i,i)^2) * lend_hat(i,i)^2 - ...
        (1/(2*len_hat(i,i)^2))* B(:,i)' * WC_nbTC_bbT(:,i);
end
Lambda = sparse(Lambda);


%% Find 'Tau' and 'mu' such that: Tau * gamma = mu

% Initializing Tau and mu
Tau = zeros(Caxes*Cnodes,alpha);
mu = zeros(Caxes*Cnodes,1);

for i = 1:Cnodes
    Tau(((i-1)*Caxes+1):(i*Caxes), :) = ...
        LN * (C_s' * diag(C_sb*MinvRb(:,i)) ...
        - C_nbTC_bbT * diag(C_bb*MinvRb(:,i))*Lambda ...
        + C_s' * diag(C_ss*MinvRs(:,i)));
    
    mu(((i-1)*Caxes+1):(i*Caxes), 1) = ...
        RHS(:,i) + LN * C_nbTC_bbT * diag(C_bb*MinvRb(:,i))*tau;
end



UpperBoundGamma = 20000; % Maximum allowed value for gamma
options = optimoptions('lsqlin','Algorithm','interior-point',...
    'Display','off');
[MyControlGamma, rnorm] = lsqlin(Tau, mu, ...
    -sparse(eye(size(Tau,2))), zeros(size(Tau,2),1), ... % inequality constraints
    [],[],[],UpperBoundGamma*ones(size(Tau,2),1),[],options);

tension = str_len_hat*MyControlGamma;
