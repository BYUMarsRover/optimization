function cost = rate3(x,fArm,mp,tries,botx,boty,fronty,frontz)

arm = fArm(x,mp);

ws_xp = 1.2;
ws_xn = .45;
ws_yp = .35;
ws_yn = -ws_yp;
ws_zn = 0;
ws_zp = 1.5;

sbx = (ws_xp - ws_xn) / botx;
sby = (ws_yp - ws_yn) / boty;

botx_offset = ws_xn;
boty_offset = ws_yn;
botz_offset = 0;

sfy = (ws_yp - ws_yn) / fronty;
sfz = (ws_zp - ws_zn) / frontz;

frontx_offset = ws_xp;
fronty_offset = ws_yn;
frontz_offset = ws_zn;

bot_points = zeros(3,botx,boty);
front_points = zeros(3,fronty,frontz);

for il = 1:botx
    for jl = 1:boty
        
        bot_points(1,il,jl) = sbx/2 + sbx*(il-1) + botx_offset;
        bot_points(2,il,jl) = sby/2 + sby*(jl-1) + boty_offset;
        bot_points(3,il,jl) = botz_offset;
        
        %plot3(bot_points(1,il,jl),bot_points(2,il,jl),bot_points(3,il,jl),'bo')
        
    end
end

for il = 1:fronty
    for jl = 1:frontz
        
        front_points(1,il,jl) = frontx_offset;
        front_points(2,il,jl) = sfy/2 + sfy*(il-1) + fronty_offset;
        front_points(3,il,jl) = sfz/2 + sfz*(jl-1) + frontz_offset;
        
        %plot3(front_points(1,il,jl),front_points(2,il,jl),front_points(3,il,jl),'bo')
        
    end
end

qs_bot = zeros(arm.n,botx,boty);
qs_front = zeros(arm.n,fronty,frontz);

ERR_bot = zeros(botx,boty);
ERR_front = zeros(fronty,frontz);

EF_bot = zeros(botx,boty);
EF_front = zeros(fronty,frontz);

q0 = zeros(1,arm.n);

for il = 1:botx
    for jl = 1:boty
        
        for kl = 1:tries
            
            if kl == 1
                q_try = q0;
            else
                q_try = rand(1,arm.n);
            end
            
            T = transl(bot_points(1,il,jl),bot_points(2,il,jl),bot_points(3,il,jl))*trotx(-pi);
            [qs(kl,:), err(kl), exitflag(kl)] = arm.ikcon(T,q_try);
            
        end
        
        [val,pos] = min(err);
        
        qs_bot(:,il,jl) = qs(pos,:)';
        ERR_bot(il,jl) = err(pos);
        EF_bot(il,jl) = exitflag(pos);
        
    end
end

for il = 1:fronty
    for jl = 1:frontz
        
        for kl = 1:tries
            
            if kl == 1
                q_try = q0;
            else
                q_try = rand(1,arm.n);
            end
            
            T = transl(front_points(1,il,jl),front_points(2,il,jl),front_points(3,il,jl))*troty(pi/2);
            [qs(kl,:), err(kl), exitflag(kl)] = arm.ikcon(T,q_try);
        end
        
        [val,pos] = min(err);
        
        qs_front(:,il,jl) = qs(pos,:)';
        ERR_front(il,jl) = err(pos);
        EF_front(il,jl) = exitflag(pos);
        
    end
end

L = sum(abs(x));

error = sum(sum(abs(ERR_bot))) / (botx*boty) + sum(sum(abs(ERR_front))) / (fronty*frontz);

cost = error*L;

end