% P = @(g,S)(1-g^S);
figure
g = linspace(0,1,100);

P = zeros(5,length(g));

for S = 0:4
    P(S+1,:) = 1-g.^(S+1);
end

plot(g,P)

xlabel('r_h(k)'),ylabel('P_{H_{h,i}}(k)')
legend('S_{o_i}(k) = 0','S_{o_i}(k) = 1','S_{o_i}(k) = 2','S_{o_i}(k) = 3','S_{o_i}(k) = 4','Location','Best')
arrow([.45,.35],[.75,.9])
text(.4,.45,'S_{o_i}(k)')
text(.4,.1,'P_{H_{h,i}}(k) = 1 - r_h(k) \^ (S_{o_i}(k) + 1)')





