function [CN, lambda] = kproperties(K)
sigma = eig(K'*K);
sigma1 = max(sigma);
sigma2 = min(sigma);
CN = sigma1/sigma2;
H = (K^-1)';
Lambda = K.*H;
lambda = Lambda(1);