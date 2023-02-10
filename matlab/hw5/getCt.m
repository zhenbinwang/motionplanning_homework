function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    Ct = zeros(2*n_seg*4,(n_seg+1)*4);

    for i=1:4
        Ct(i,i) = 1;
    end

    for i=1:4
        Ct(2*n_seg*4-i+1,2*4+n_seg-i) = 1;
    end

    for i=1:n_seg-1
        Ct(4+8*(i-1)+1,4+i) = 1;
        Ct(4+8*(i-1)+5,4+i) = 1;
    end

    for i=1:n_seg-1
        for j=1:3
            Ct(4+8*(i-1)+1+j,8+(n_seg-1)+(i-1)*3+j) = 1;
            Ct(4+8*(i-1)+5+j,8+(n_seg-1)+(i-1)*3+j) = 1;
        end
    end
end