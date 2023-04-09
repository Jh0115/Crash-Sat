clear all
close all
clc

Nmax = 100;
test_num = 2000;

%basic equation
mu = 1.96;
n = 1:Nmax;
mu_samp = mu./sqrt(n);

%array method (but done many times for clarity
for jj = 1:test_num
    for ii = 1:Nmax
        x_new = randn();
    
        % array method
        x(ii) = x_new;
        mu_arr(jj,ii) = (sqrt(sum((x(1:ii)-mean(x)).^2)))*1.96/ii;
    
    end
end

%scalar update method
for jj = 1:test_num
    x = randn;
    var_prev = var(x);
    avg_x_prev = mean(x);
    var_calc = var_prev;
    for ii = 2:Nmax
        x = randn();
        N = ii;

        %this function outputs sample variance
        avg_x_new = avg_x_prev+(x-avg_x_prev)/N;
        var_calc(ii) = ((N-2)*var_calc(ii-1)+(x-avg_x_new)*(x-avg_x_prev))/(N-1);
    
        sig_samp_calc(jj,ii) = sqrt(var_calc(ii))/sqrt(N);
    
        avg_x_prev = avg_x_new;
        var_prev = var_calc(ii);
            
    end
end

mu_sca = 1.96*sig_samp_calc;

figure()
hold on
plot(mu_samp)
plot(mean(mu_arr,1))
plot(mean(mu_sca,1))
legend("Actual","Array","Scalar")
