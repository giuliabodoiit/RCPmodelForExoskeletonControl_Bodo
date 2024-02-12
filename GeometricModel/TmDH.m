function TmDH = TmDH(mDH)
a = mDH(1);
alp = mDH(2);
d = mDH(3);
th = mDH(4);
%UNTITLED4 Summary of this function goes here
% Detailed explanation goes here
TmDH = [cos(th),-sin(th),0,a;sin(th)*cos(alp),cos(th)*cos(alp),-sin(alp),-d*sin(alp);...
sin(th)*sin(alp),cos(th)*sin(alp),cos(alp),d*cos(alp);0,0,0,1];
end

