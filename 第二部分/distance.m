function aflen = distance( XX,pathCount,lines,S,T)
aflen=0;

    Pstart = S;
    a=XX(1,1);
    Pend = lines(1,1:2) + (lines(1,3:4)-lines(1,1:2))*a;
    for k=1:pathCount
        aflen = aflen + sqrt(sum((Pend-Pstart).^2));
        Pstart=Pend;
        if k<pathCount
            a=XX(1,k+1);
            Pend = lines(k+1,1:2) + (lines(k+1,3:4)-lines(k+1,1:2))*a;
        end
    end
    Pend = T;
    aflen = aflen + sqrt(sum((Pend-Pstart).^2));


