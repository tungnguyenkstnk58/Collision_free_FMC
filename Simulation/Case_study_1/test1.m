clear 
clc
close all

ob1 = [1.6;1;1];
ob2 = [1.5;1;1];
ob = [ob1,ob2];
pos = [1,1,0];
n = size(ob);
no = n(2);
mr = 4.0*ones(4,1);

for i = 1:1:no
    if ob(1,i) == pos(1)
       dis_y = ob(2,i) - pos(2);
       if dis_y > 0 && dis_y < 4.0 && dis_y < mr(3)
          mr(3) = dis_y;
       elseif dis_y < 0 && dis_y > -4.0 && abs(dis_y) < mr(4)
          mr(4) = dis_y;
       end
    end
    %
    if ob(2,i) == pos(2)
       dis_x = ob(1,i) - pos(1);
       if dis_x > 0 && dis_x < 4.0 && dis_x < mr(1)
          mr(1) = dis_x;
       elseif dis_x < 0 && dis_x > -4.0 && abs(dis_x) < mr(2)
          mr(2) = dis_x;
       end
    end

end

mr
