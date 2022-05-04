clear all
close all

% [Null1,Null2,Null4,v_c]=xlsread('Cartel1.xlsx','value_v_computed');
% [Null1,Null2,Null4,v_b]=xlsread('Cartel1.xlsx','value_v_bag');
v_c=xlsread('Cartel1.xlsx','value_v_computed','J:J');
v_b=xlsread('Cartel1.xlsx','value_v_bag','J:J');

%%
var_c=var(v_c);
var_b=var(v_b);


v_c_bias=v_c-mean(v_c);
v_b_bias=v_b-mean(v_b);

v_c_var=v_c_bias./var_c;
v_b_var=v_b_bias./var_b;

% w_bias=inv((v_c_bias'*v_c_bias))*v_c_bias'*v_b_bias;
% w_var=inv((v_c_var'*v_c_var))*v_c_var'*v_b_var;
w=inv(v_c'*v_c)*v_c'*v_b;

 %%
figure
plot(v_b,v_c)
hold on
v_b1=w*v_c;
plot(v_b1,v_c);
figure
plot(v_b_bias,v_c_bias)
figure
plot(v_b_var,v_c_var)


%%