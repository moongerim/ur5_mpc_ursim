clear all;
close all;
clc;
time_to_stop = zeros(1,10);
error_tp = zeros(10,10);
acceleration = zeros(10,10);
for f = 1:10
    filename = sprintf('data_acceleration_%i.csv', f);
    data = load(filename);
    joint_poses = data(:,1:6);
    joint_vels = data(:,7:12);
    given_joint_vels = data(:,13:18);
    ctp = data(:,19:48);
    ctv = data(:,49:78);
    time = data(:,79);
    len = length(joint_poses);

    % compute linear velocity
    linear_vell = zeros(len,10)+1;
    for l = 1:len
        for p=0:9
            index_1 = p*3+1;
            index_2 = index_1+1;
            index_3 = index_1+2;
            linear_vell(l,p+1) = sqrt(ctv(l,index_1)*ctv(l,index_1)+ctv(l,index_2)*ctv(l,index_2)+ctv(l,index_3)*ctv(l,index_3));
        end
    end
     
    % robot starts to move:
    for i=1:len
        if linear_vell(i,1)>0 && given_joint_vels(i,1)==1
            n = i;
            break;
        end
    end

    % a script provides 0 and real robot joint velocity decreases:
    for i = n:len
        if linear_vell(i,1)<1 && given_joint_vels(i,1)==0
            s = i;
            break;
        end
    end

    % robot stops:
    for i=s:len
        if linear_vell(i,1)==0 && linear_vell(i,2)==0 && linear_vell(i,3)==0 && linear_vell(i,4)==0 && linear_vell(i,5)==0 ...
            && linear_vell(i,6)==0 && linear_vell(i,7)==0 && linear_vell(i,8)==0 && linear_vell(i,9)==0 && linear_vell(i,10)==0
            e = i;
            break;
        end
    end

    % Time to stop
    time_to_stop(f) = time(e)-time(s)
    % Test point errors:
    error_tp(f,:) = zeros(1,10);
    for p=0:9
        index_s = p*3+1
        index_e = index_s+2
        error_tp(f,p+1) = norm(ctp(e,index_s:index_e)-ctp(s,index_s:index_e))
    end
    
    acceleration(f,:) = error_tp(f,:)/(time_to_stop(f)^2)
end

new_file(:,1) = time_to_stop;
new_file(:,2:11) = error_tp;
new_file(:,12:21) = acceleration;

N = array2table(new_file, 'VariableNames', {'T', 'E_1', 'E_2', 'E_3', 'E_4', 'E_5', 'E_6', 'E_7', 'E_8', 'E_9', 'E_10', 'A_1', 'A_2', 'A_3', 'A_4', 'A_5', 'A_6', 'A_7', 'A_8', 'A_9', 'A_10'})
