clear
close all
clc

dt = 0.1; % step size
ts = 25; % simulation time
tspan  = 0:dt:ts; % time span


%% Vehicle parameters

syms a l th2 w1 w2 psi th1 real

phi=0;
P=[phi phi];
A=[a a];
dx=[-l l];
dy=[0 0];
T=[th1 th2];

%% Hallamos Q y W del robot

for i=1:size(A,2)
 Q(i,:)=fn_wi(A(i),dx(i),dy(i),P(i),T(i));
end

simplify(Q) %Desplegamos la matriz simbólica Q


% se halla el inverso de Q que es W (omega=Q*zeta y zeta=W*omega) :

n=size(Q,1);
if n==3
    W=inv(Q); %Cuando es cuadrada (3 ruedas)

% en el caso de que Q no sea invertible (que es el caso para 4 ruedas) :
elseif n>3
    W=inv(Q'*Q)*Q'; 

% menos de 3 ruedas    
elseif n<3
        W=Q'*inv(Q*Q'); 

end

simplify(W) %Desplegamos la matriz simbólica W

a = 0.2; % rad. rueda
l = 0.5; % dist. rueda - {B}
d = l*2; % car width

Q=eval(Q)
W=eval(W)


%% Set-Points

% initial position
beta_0=0;
eta(:,1) = [-2;-2;beta_0];

% desired position vector
x = -4;
y = -4;
beta = pi/2;

eta_d = [x;y;beta];

% desired velocity vector
x_dot = 0;
y_dot = 0;
beta_dot = 0;

eta_d_dot = [x_dot;y_dot;beta_dot];

lambda = diag([1,1,2]);



%% Fase 1: control de posicion del body

J_psi_p1=rot_z(psi); %para la fase 1: control de posicion
J_psi_pinv1=inv(J_psi_p1); % para la fase 1

%% Fase 2: control de orientacion del body





%% funciones para hallar Q
function Q=fn_wi(a,dx,dy,phi,teta)

    rot_phi=1/a*[1 tan(phi)]; % rotacion imputable al roller

    rot_rueda_body=rot_z(teta); % cambio de sc rueda-->body

    cambio_body_centro=[1 0 -dy;
                        0 1 dx]; % cambio de sc body-->centro del body

    Q=rot_phi*rot_rueda_body*cambio_body_centro;
end

function R=rot_z(psi)
    R= [cos(psi) sin(psi);
            -sin(psi) cos(psi)];
end


function A=eval_function(W,t1,t2)
    th1=t1;
    th2=t2;
    A=eval(W);
end


function R=protz(psi)
    R= [cos(psi) sin(psi) 0;
        -sin(psi) cos(psi) 0;
            0       0      1];

end