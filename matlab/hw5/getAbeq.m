function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    Aeq_start(1,1) = factorial(0);
    Aeq_start(2,2) = factorial(1);
    Aeq_start(3,3) = factorial(2);
    Aeq_start(4,4) = factorial(3);

    beq_start = start_cond';
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    t = ts(end);
    for k=1:4
        for i=1+k-1:n_order+1
            Aeq_end(k,n_all_poly-n_order-1+i) = factorial(i-1)*t.^(i-k)/factorial(i-k);
        end
    end
    beq_end = end_cond';
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp

%     t=0;
%     for i=1:n_seg-1
%         t= t+ts(i);
%         for j=1:n_order+1
%            
%             Aeq_wp(i,j)= t^(n_order+1-j);
%         end
%     end

    for i=1:n_seg-1
        for j=1:n_order+1
            Aeq_wp(i,j+(i-1)*(n_order+1))= ts(i).^(j-1);
        end
    end
      
    beq_wp = waypoints(2:end-1);
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %#####################################################
    k = 1;
    t_start = 0;
    for i=1:n_seg-1
        for j=1:n_order+1
            Aeq_con_p(i,j+(i-1)*(n_order+1)) = factorial(j-1)*ts(i)^(j-k)/factorial(j-k);
            Aeq_con_p(i,j+i*(n_order+1))= -factorial(j-1)*t_start^(j-k)/factorial(j-k);
        end
        
    end

    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    k = 2;
    for i=1:n_seg-1
        for j=k:n_order+1
            Aeq_con_v(i,j+(i-1)*(n_order+1)) = factorial(j-1)*ts(i)^(j-k)/factorial(j-k);
            Aeq_con_v(i,j+i*(n_order+1)) = -factorial(j-1)*t_start^(j-k)/factorial(j-k);
        end
    end

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    k = 3;
    for i=1:n_seg-1
        for j=k:n_order+1
            Aeq_con_a(i,j+(i-1)*(n_order+1)) = factorial(j-1)*ts(i)^(j-k)/factorial(j-k);
            Aeq_con_a(i,j+i*(n_order+1)) = -factorial(j-1)*t_start^(j-k)/factorial(j-k);
        end
    end
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    k = 4;
    for i=1:n_seg-1
        for j=k:n_order+1
            Aeq_con_j(i,j+(i-1)*(n_order+1)) = factorial(j-1)*ts(i)^(j-k)/factorial(j-k);
            Aeq_con_j(i,j+i*(n_order+1)) = -factorial(j-1)*t_start^(j-k)/factorial(j-k);
        end
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end