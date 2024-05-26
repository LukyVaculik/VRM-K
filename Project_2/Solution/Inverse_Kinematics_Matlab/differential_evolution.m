%% INFO
%%V1.0, Differential Evolution algorithm Matlab, 10.5.2024, Lukáš Vaculík
%% RESOURCES
% OPTIMIZATION METHODS I Slides, BUT Course FOA-K - Chapter 9. Population Methods
% Ing. Jakub Kůdela, Ph.D
%% CODE
function [fmin,xs,iter,pops] = differential_evolution(f,x0,max_iter,epsilon,lb,ub)
n = length(x0); 
m = 30; 
w = 0.8; p = 0.9;
iter = 0;
xs = x0; fmin = f(x0);
pop = [x0+randn(n,m)]; 
pops = pop;
for i=1:m
    if ~isempty(lb)
        pop(:,i) = enforce_bounds(pop(:,i),lb,ub);
    end
    val(i) = f(pop(:,i));
end
for k=1:max_iter
    children = zeros(n,m);
    for i=1:m
        perm = randperm(m,3);
        z = pop(:,perm(1)) + w*(pop(:,perm(2)) - pop(:,perm(3)));
        j = randperm(n,1);
        children(:,i) = pop(:,i);
        for d = 1:n
            if d == j || p < rand(1)
                children(d,i) = z(d);
            end
        end
        if ~isempty(lb)
            children(:,i) = enforce_bounds(children(:,i),lb,ub);
        end
    end
    for i=1:m
        val_child = f(children(:,i));
        if val_child < val(i)
            pop(:,i) = children(:,i);
            val(i) = val_child;
            if val_child < fmin
                fmin = val_child;
                xs(:,end+1) = children(:,i);
            end
        end
    end
    pops(:,:,end+1) = pop;
    iter = iter + 1;
    if (fmin < epsilon)
        break;
    end
end

end

function val = enforce_bounds(x,lb,ub)
%simple projection
val = min(max(lb,x),ub);
end


