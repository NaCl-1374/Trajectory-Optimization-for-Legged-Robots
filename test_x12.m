clc;
clear;
warning off
addpath('E:\Matlab\casadi-matlab')
import casadi.*

%%
N = 16*4; % N = 11
T = 0.5*4; % T = 0.22
dt_val = repmat(T/(N),1,N)';
% cs_val = [repmat([1 1 1 1]', 1, 3) repmat([1 1 1 1]', 1, 2) repmat([0 0 0 0]', 1, 8) repmat([1 1 1 1]', 1, 3)]';
% cs_val = ~[repmat([1 1 1 1]', 1, 3) repmat([1 1 1 1]', 1, 2) repmat([0 0 0 0]', 1, 8) repmat(~[1 1 1 1]', 1, 3)]';
cs_val = [repmat([1 1 1 1]', 1, 3*4) repmat([1 1 1 1]', 1, 2*4) repmat([0 0 0 0]', 1, 7*4) repmat([1 1 1 1]', 1, 4*4)]';

cs_TD_val = zeros(4,N-1)';
%% %状态的权重

weight.QX = [10 10 10, 10 10 10, 10 10 10, 10 10 10 ]';
weight.QN = [10 10 10, 50 50 100, 10 10 10, 10 10 10 ]';
weight.Qc = [0.001 0.001 0.001]';
weight.Qf = [0.0001 0.0001 0.001]';
%%  %物理参数

Body.m = 9;%机器人质量
%机身惯量
Body.Ib = [0.07 0     0;
    0    0.26   0;
    0    0     0.242];%转动惯量矩阵
Body.length_body=0.38;
Body.width_body=0.22;
Body.hipPos=[0.2,0.2,-0.2,-0.2;
    0.1,-0.1,0.1,-0.1;
    0,  0,   0, 0];
world.g = 9.8;%重力加速度
world.mu=0.4;%摩擦系数
%%  构造微分方程

Xk=SX.sym('Xk', 12, 1);
n_state=size(Xk,1);
Fk=SX.sym('Uk', 12, 1);
n_F=size(Fk,1);
Rk=SX.sym('Rk', 12, 1);
n_r=size(Rk,1);
%%  计算微分方程
I3=eye(3);
Rbody=rotsb(Xk(1:3));
cy = cos(Xk(3));
sy = sin(Xk(3));
cp = cos(Xk(2));
sp = sin(Xk(2));

R_yaw =[cy sy 0;
        -sy cy 0;
        0 0 1];%世界到机身
R_w=[cy/cp,sy/cp,0;
    -sy,cy,0;
    cy*sp/cp,sy*sp/cp,1];
Ig = Rbody*Body.Ib*Rbody';
Ig_inv=Ig\I3;


A = [zeros(3) zeros(3) R_yaw zeros(3)  ;
    zeros(3) zeros(3) zeros(3) I3 ;
    zeros(3) zeros(3) zeros(3) zeros(3);
    zeros(3) zeros(3) zeros(3) zeros(3) ;
    ];%状态矩阵

AA=A;
AA(1:3,7:9)=R_w;

B=[zeros(3)           zeros(3)           zeros(3)            zeros(3);
    zeros(3)           zeros(3)           zeros(3)            zeros(3);
    Ig_inv*Skew(Rk(1:3)) Ig_inv*Skew(Rk(4:6)) Ig_inv*Skew(Rk(7:9))  Ig_inv*Skew(Rk(10:12));
    I3/Body.m   I3/Body.m   I3/Body.m    I3/Body.m;];%控制矩阵
g=zeros(12,1);
g(12)=-world.g;
dotX=A*Xk+B*Fk+g;
%%  定义函数
f=Function('f',{Xk;Fk;Rk},{dotX},{'input_states','control_inputs','foot_input'},{'dotX'});

% X_init = [0;0.0;0; 0.0;0.0;0.5 ;0;0;0; 0;0;0;-9.8];%初始状态变量
% f(X_init,zeros(12,1),zeros(12,1))%测试函数正常否

%%  构造代价和约束 变量定义
X = SX.sym('X', n_state, N+1); % N+1步状态
F = SX.sym('F', n_F, N); % N步内的控制
r = SX.sym('r', n_r, N); % N步内的控制

RefX = SX.sym('RefX', n_state, N+1); % N步内的控制输出
RefF = SX.sym('RefF', n_F, N); % N步内的控制输出
Refr = SX.sym('Refr', n_r, N); % N步内的控制输出
ContactState=SX.sym('ConState', 4, N);
obj=0;
%%  构造代价和约束 变量定义
mu_inv = 1.0/world.mu;
%摩擦约束
f_block =[ mu_inv, 0,  -1.0;
    -mu_inv, 0,  -1.0;
    0,  mu_inv, -1.0;
    0, -mu_inv, -1.0;];

kin_box_x = 0.05;
kin_box_y = 0.05;
kin_box_z = 0.3;

Kin_block =[ 1, 0,  0,-kin_box_x;
    -1, 0,  0,-kin_box_x;
    0, 1, 0,-kin_box_y;
    0, -1, 0,-kin_box_y;
    0, 0, 1,0.06;
    0, 0, -1,-kin_box_z];

hipPos=Body.hipPos;
hipPos(3,:)=[-0.2,-0.2,-0.2,-0.2];
Phip_swing=reshape(hipPos,[],1);
%%  约束暂存变量定义 %初态约束
%初态约束
defect_init=RefX(:,1)-X(:,1);%12*1 eq

defect_state=SX.zeros(12*(N+1),1);%12(N+1)*1 eq
defect_FootOnGround=SX.zeros(4*(N),1);%4(N)*1 eq
defect_footStance=SX.zeros(12*(N),1);%(3*4)(N)*1 eq
n_equa_c=12+12*(N+1)+4*(N)+12*(N);
%共
defect_legLimits=SX.zeros(24*(N),1);%(4*6)(N)*1
defect_footforce=SX.zeros(16*(N),1);%(4*4)(N)*1 xy摩擦约束4个
defect_ForceNormal=SX.zeros(N,1);% (N)*1
defect_footswing=SX.zeros(4*(N),1);%4(N)*1
n_inequa_c=24*(N)+16*(N)+N+4*(N);
%%	约束和代价计算
for k = 1:N     
    %%	代价计算    
    Xk=X(:,k);
    Fk=F(:,k);
    rk=r(:,k);
    Pk=repmat(Xk(4:6),4,1)+rk;
    ContactStatek=ContactState(:,k);
    dtk=dt_val(k);
    
    X_err = Xk - RefX(:,k);                                         % 基座状态误差
    pf_err = repmat(Xk(4:6),4,1) + Phip_swing - Pk;                      %  悬空时约束foot位置
    U_err = Fk - RefF(:,k);                                         % GRF 误差
    obj = obj + (X_err'*diag(weight.QX)*X_err+...                     % 误差求和
        pf_err'*diag(repmat(weight.Qc,4,1))*pf_err+...
        U_err'*diag(repmat(weight.Qf,4,1))*U_err)*dtk;
    %% 约束计算
    %状态约束
    %runge kutta method
%     k1 = f(Xk,Fk,Pk);   % new
%     k2 = f(Xk + dtk/2*k1,Fk,Pk); % new
%     k3 = f(Xk + dtk/2*k2,Fk,Pk); % new
%     k4 = f(Xk + dtk*k3,Fk,Pk); % new
%     st_next_RK4=Xk +dtk/6*(k1+2*k2+2*k3+k4); % new
%     defect_state((k-1)*12+1:(k-1)*12+12)=X(:,k+1)-(st_next_RK4);
        defect_state((k-1)*12+1:(k-1)*12+12)=X(:,k+1)-(Xk+f(Xk,Fk,rk)*dtk);

    
    %法向力大于0 不等式
    defect_ForceNormal(k)=-dot(Fk,repmat([0;0;1],4,1));
    %结合法向力大于0，摩擦约束来约束摆动中力为0 和最大力 不等式
    defect_footswing((k-1)*4+1:(k-1)*4+4)=Fk([3,6,9,12])-ContactStatek.*repmat(1000,4,1);
    for leg=1:4
        xyz_idx = 3*(leg-1)+1:3*(leg-1)+3;
        %脚在地上约束 0是此时地面高度等式
        defect_FootOnGround((k-1)*4+leg)=ContactStatek(leg)*Pk(3*(leg-1)+3);
        %限制腿长 限制范围不等式
        Rbody=rotsb(Xk(1:3));
        Phip=Rbody*Body.hipPos+Xk(4:6);
        p_rel = (Pk(xyz_idx) - Phip(:,leg));%hip->足端
        defect_legLimits((k-1)*24+(leg-1)*6+1:(k-1)*24+(leg-1)*6+6)= Kin_block*[p_rel;1];
        %接触中脚不滑动
        if (k < N)
            Pk1=repmat(X(4:6,k+1),4,1)+r(:,k+1);
            defect_footStance((k-1)*12+(leg-1)*3+1:(k-1)*12+(leg-1)*3+3)=ContactStatek(leg)*(Pk1(xyz_idx)-Pk(xyz_idx));
        end
        %摩擦约束 不等式
        defect_footforce((k-1)*16+(leg-1)*4+1:(k-1)*16+(leg-1)*4+4)=f_block*Fk(xyz_idx);
    end
end
%%	约束生成
g=[defect_init;defect_state;defect_FootOnGround;defect_footStance;...
    defect_legLimits;defect_footforce;defect_ForceNormal;defect_footswing];
display_str=['等式约束数量',num2str(n_equa_c),'   不等式约束数量',num2str(n_inequa_c)];
disp(display_str);
%%	终端 cost
X_err = X(:,end)-RefX(:,end);    % 终端 cost
obj = obj + X_err'*diag(weight.QN)*X_err;
%%	构造问题和问题变量
OPT_variables = [reshape(X,n_state*(N+1),1);reshape(F,n_F*N,1);reshape(r,n_r*N,1)];
OPT_Param = [reshape(RefX,n_state*(N+1),1);reshape(RefF,n_F*N,1);reshape(Refr,n_r*N,1);reshape(ContactState,4*N,1)];
nlp_prob =struct('f', obj, 'x',OPT_variables,'p',OPT_Param, 'g',g);
%%  优化设置
opts_setting=struct;
opts_setting.ipopt.max_iter=3000;
opts_setting.ipopt.print_level=0;
opts_setting.ipopt.acceptable_tol=1e-4;
opts_setting.ipopt.acceptable_obj_change_tol=1e-6;
opts_setting.ipopt.tol=1e-4;
opts_setting.ipopt.nlp_scaling_method='gradient-based';
opts_setting.ipopt.constr_viol_tol=1e-3;
opts_setting.ipopt.fixed_variable_treatment='relax_bounds';
% opts_setting.ipopt.
% opts_setting.ipopt.
% opts_setting.ipopt.
% opts_setting.ipopt.
%% 构造求解器
solver = nlpsol('solver', 'ipopt', nlp_prob,opts_setting);
%%	约束上下界面 args
args.lbg(1:n_equa_c) = 0;  % -1e-20  % Equality constraints
args.ubg(1:n_equa_c) = 0;  % 1e-20   % Equality constraints

args.lbg(n_equa_c+1 : n_equa_c+ n_inequa_c) = -inf; % inequality constraints
args.ubg(n_equa_c+1 : n_equa_c+ n_inequa_c) = 0; % inequality constraints
%%	决策变量上下界面 args
%%  状态上边界
tempub=[Body.m*world.g*world.mu*6; Body.m*world.g*world.mu*6 ;1000];
args.ubx=[];
UBx=[pi*3*ones(3,1);10*ones(2,1);1;...
    pi*3*ones(3,1);40*ones(3,1)];
UBx=repmat(UBx,N+1,1);
UBf=[tempub;tempub;tempub;tempub];
UBf=repmat(UBf,N,1);
UBp=repmat([0.4;0.4;inf],4,1);
UBp=repmat(UBp,N,1);
args.ubx=[args.ubx;UBx;UBf;UBp];
%%  状态下边界
templb=[-Body.m*world.g*world.mu*6; -Body.m*world.g*world.mu*6 ;0];
args.lbx=[];
LBx=[-pi*3*ones(3,1);-10*ones(2,1);0;...
    -pi*3*ones(3,1);-40*ones(3,1)];
LBx=repmat(LBx,N+1,1);
LBf=[templb;templb;templb;templb];
LBf=repmat(LBf,N,1);
LBp=repmat([-0.4;-0.4;-inf],4,1);
LBp=repmat(LBp,N,1);
args.lbx=[args.lbx;LBx;LBf;LBp];
%%
q_init_val = [0 0 0 0.0 0 0.2]';
qd_init_val = [0 0 0 0 0 0]';

q_term_ref = [2*pi 0 0 1 0 0.9 ]';
qd_term_ref = [0 0 0, 0 0 0]';

c_init_val = repmat(q_init_val(4:6),4,1)+...
    diag([1 1 1, 1 -1 1, -1 1 1, -1 -1 1])*repmat([0.2 0.1 -q_init_val(6)],1,4)';

c_ref = diag([1 1 1, 1 -1 1, -1 1 1, -1 -1 1])*repmat([0.2 0.1 -0.2],1,4)';
f_ref = zeros(12,1);

%% set parameter values
for i = 1:6
    Xref_val(i,:)   = linspace(q_init_val(i),q_term_ref(i),N+1);
    Xref_val(6+i,:) = linspace(qd_init_val(i),qd_term_ref(i),N+1);
end
% Z向抛物线
a=[Xref_val(4,1),Xref_val(4,N/2),Xref_val(4,N)];%x
b=[q_init_val(6),q_term_ref(6),q_init_val(6)+0.0];%z
Xref_val(6,:) =interp1(a,b,Xref_val(4,:),'spline'); 
Uref_val=zeros(24,N);
r_ref=zeros(12,N);
for leg = 1:4
    for xyz = 1:3
        Uref_val(3*(leg-1)+xyz,:)= Xref_val(xyz+3,1:end-1) +c_ref(3*(leg-1)+xyz);%F 
        r_ref(3*(leg-1)+xyz,:)= c_ref(3*(leg-1)+xyz);%
        Uref_val(12+3*(leg-1)+xyz,:) = f_ref(xyz).*ones(1,N);%P
    end
end
F_ref=Uref_val(13:24,:);
% P_ref=Uref_val(1:12,:);
% for i=1:N-1
%     cube_animate(Xref_val(:,i),i,P_ref(:,i),~cs_val(i,:),[0;0;0;0],...
%         F_ref(:,i),3,[],[],[],[],[],[],dt_val);
% % pause(0.01);
% end
% prompt = 'continue ? ';
% Continue=input(prompt);

args.p=[reshape(Xref_val,n_state*(N+1),1);reshape(F_ref,n_F*N,1);reshape(r_ref,n_r*N,1);reshape(cs_val',4*N,1)];

args.x0=[reshape(Xref_val,n_state*(N+1),1);reshape(F_ref,n_F*N,1);reshape(r_ref,n_r*N,1)];
sol=solver('x0',args.x0,'lbx', args.lbx,'ubx', args.ubx,'lbg', args.lbg,'ubg', args.ubg,'p',args.p);
x_li=sol.x(1:n_state*(N+1));
x_li=reshape(full(x_li),n_state,(N+1));

f_li=sol.x(n_state*(N+1)+1:n_state*(N+1)+n_F*N);
f_li=reshape(full(f_li),n_F,N);

r_li=sol.x(n_state*(N+1)+n_F*N+1:n_state*(N+1)+n_F*N+n_r*N);
r_li=reshape(full(r_li),n_F,N);
p_li=r_li+repmat(x_li(4:6,1:end-1),4,1);
figure(5);
plot3(x_li(4,:),x_li(5,:),x_li(6,:));
pic_num = 1;%保存gif用
time=['NLP','_',datestr(datetime('now'),'yyyy-mm-dd-HH-MM'),'_Animated.gif'];
for i=1:N
    cube_animate(x_li(:,i),i,p_li(:,i),~cs_val(i,:),[0;0;0;0],...
        f_li(:,i),3,[],[],[],[],[],[-30,60],dt_val,[]);
%         frame = getframe(figure(3));
%     [A,map]=rgb2ind(frame2im(frame),256);
%     if pic_num==1
%         imwrite(A,map,time,'gif','LoopCount',Inf,'DelayTime',dt_val(i));
%     else
%         imwrite(A,map,time,'gif','WriteMode','append','DelayTime',dt_val(i));
%     end
%     pic_num=pic_num+1;
pause(0.1);
end
%% 工具函数
function rotxm=rotx(theta)
s=sin(theta);
c=cos(theta);
% rotxm=[1,0,0;
%     0,c,s
%     0,-s c]';
rotxm=[1,0,0;
    0,c,-s
    0,s c];
end

function rotym=roty(theta)
s=sin(theta);
c=cos(theta);
% rotym =[c,0,-s;
%     0,1,0;
%     s,0,c]';
rotym =[c,0,s;
    0,1,0;
    -s,0,c];
end

function rotzm=rotz(theta)
s=sin(theta);
c=cos(theta);

% rotzm=[c,s,0;
%     -s,c,0;
%     0,0,1]';
rotzm=[c,-s,0;
    s,c,0;
    0,0,1];
end
%Rsb
function R=rotsb(theta)
% R=rotx(theta(1))*roty(theta(2))*rotz(theta(3));
R=rotz(theta(3))*roty(theta(2))*rotx(theta(1));

end

function s=Skew(in)
% s=zeros(3,3);
% s(1,2)=-in(3);
% s(1,3)=in(2);
% s(2,3)=-in(1);
% s(2,1)=in(3);
% s(3,1)=-in(2);
% s(3,2)=in(1);
s = [0 -in(3) in(2);
    in(3) 0 -in(1);
    -in(2) in(1) 0];
end

