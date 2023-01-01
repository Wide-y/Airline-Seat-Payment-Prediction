num_frames = 101;
dd = 1/(num_frames-1);
P_0 = [2;3;0];
s = 0:dd:1;

%% Iterations

for frame=1:length(s)
    P_t = object + P_0 + [3*s(frame); 2*s(frame); 0];

    Ob = Obp2Tr(P_t);
    img = zeros(2*camA(2,3), 2*camA(1,3), 3);

    triangle_matrix = zeros(3,3,8);

    for i = 1:size(Rm,3)
        triangle_matrix(:,:,i) = Rm(:,:,i);
    end

    for i = 1:size(Ob,3)
        triangle_matrix(:,:,size(Rm,3)+i) = Ob(:,:,i);
    end



    for u = 1:2*camA(1,3) 
        for v = 1:2*camA(2,3) 
            % Synthesis of background
            min_distance = Crit;
            for k = 1:2
                triangle = triangle_matrix(:,:,k);
                D = triangle(:,3);
                D_N = cross(triangle(:,1)-D, triangle(:,2)-D);
                P_uv = camR/camA*[u;v;1];
                lambda = transpose(D_N)*(D-camT)/(transpose(D_N)*P_uv);
                P_w = lambda*P_uv + camT;
                M = [triangle; [1,1,1]];
                N = [P_w; 1];
                K = (transpose(M)*M)\transpose(M)*N;
                if min(K) < crit
                    continue
                elseif min_distance > sqrt(sum((P_w-camT).^2))
                    min_distance = sqrt(sum((P_w-camT).^2));
                    img(v,u,:) = [0.5; 0.5; 0.5];
                end
            end

            % Synthesis of object
            for k = 3:8
                triangle = triangle_matrix(:,:,k);
                D = triangle(:,3);
                D_N = cross(triangle(:,1)-D, triangle(:,2)-D);
                P_uv = camR/camA*[u;v;1];
                lambda = transpose(D_N)*(D-camT)/(transpose(D_N)*P_uv);
                P_w = lambda*P_uv + camT;
                M = [triangle; [1,1,1]];
                N = [P_w; 1];
                K = (transpose(M)*M)\transpose(M)*N;
                if min(K) < crit
                    continue
                elseif min_distance > sqrt(sum((P_w-camT).^2))
                    min_distance = sqrt(sum((P_w-camT).^2));
                    img(v,u,:) = Obcl(:,k-2);
                end
            end
        end
    end

