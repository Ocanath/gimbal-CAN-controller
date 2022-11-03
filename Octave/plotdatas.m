%% plot datas

figure(1)
clf
hold on
for i = 1:size(datas,1)
    plot(datas(i,:))
end
hold off


%%

for i = 1:size(datas,1)
    figure(i)
    plot(datas(i,:))
end


%%

theta = datas(11,:);
figure(1)
clf
hold on
plot(datas(1,:) + 2000)
plot(datas(2,:))
plot(datas(3,:) - 2000)


plot(-823*sin(theta/4096)+ 2000)
plot(-823*sin(theta/4096 + 120*pi/180) - 2000)
plot(-823*sin(theta/4096 - 120*pi/180))
% plot(datas(4,:))

% plot(datas(5,:)+5000)
% plot(datas(6,:)+5000)
hold off
% legend('iA','iB','iC','sector','tF','tS');
% legend('iA','iB','iC');

figure(2)
clf
hold on
% plot(datas(7,:)+4000)
% plot(datas(8,:)+4000)
plot(datas(9,:)+4000)
plot(datas(10,:))
% plot(datas(11,:)/10)
hold off
% legend('ialpha','ibeta','iq','id','theta');
% legend('iq','id');


%%


sector = datas(4,:)/100;
curA = datas(5,:);
curB = datas(6,:);

iA = zeros(1, length(curA));
iB = zeros(1, length(curA));
iC = zeros(1, length(curA));

for i = 1:length(curA)
    if(sector(i) == 1)
        iC(i)= -curA(i);
        iA(i) = curB(i);
        iB(i) = -( iC(i)+iA(i) );
    end
    if(sector(i) == 2)
        iC(i) = -curA(i);
        iB(i) = curB(i);
        iA(i) = -(iC(i)+iB(i));
    end
    if(sector(i) == 3)
        iA(i) = -curA(i);
        iB(i) = curB(i);
        iC(i) = curA(i)-curB(i);
%         iC(i) = -( -curB(i) + curA(i));
    end
    if(sector(i) == 4)
        iA(i) = -curA(i);
        iC(i) = curB(i);
        iB(i) = curA(i)-curB(i);
    end
    if(sector(i) == 5)
        iB(i) = -curA(i);
        iC(i) = curB(i);
        iA(i) = curA(i)-curB(i);
    end
    if(sector(i) == 6)
        iB(i) = -curA(i);
        iA(i) = curB(i);
        iC(i) = curA(i)-curB(i);
    end
end

figure(2)
clf
hold on
plot(iA+2000)
plot(iB-2000)
plot(iC)

f = (869-19)/4;
p = 10.2;
t = 1:1000;
plot( -800*sin(t/f*2*pi - p) )
plot( -800*sin(t/f*2*pi - p + 120*pi/180) - 2000)
plot( -800*sin(t/f*2*pi - p - 120*pi/180) + 2000)
% plot(curA)
% plot(curB)
% plot(-curA)
% plot(-curB)
% plot( -(-curA + curB))

%%

figure(1)
clf
hold on
plot(datas(1,:) + 500)
plot(datas(2,:))
plot(datas(3,:) - 500)
plot(datas(4,:)+800)
hold off

figure(2)
clf
hold on 
plot(datas(5,:))
plot(datas(6,:))
plot(datas(7,:))
hold off
