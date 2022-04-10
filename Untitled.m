x = 0:1/100:15;
y1 = 0.5*(30./sqrt(900+(400./x).^2)+1);

plot(x,y1);
xlabel('P');
ylabel('Ratio');

fileID = fopen('graph.txt', 'r');
formatSpec = '%d';
A = fscanf(fileID, formatSpec);
x = [0:0.001:1-0.001];
B = x *2
plot(x,B)
