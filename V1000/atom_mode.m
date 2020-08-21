function [T,P,rho,Ma_] = atom_mode(y)
%输入海拔高度
%输出 T P rho Ma
R0 = 6356000.0;
     R = 29.2713;
	 H0 = R0*y/(y+R0);	
     if(H0 <= 11000)
		T = 288.15 - 0.0065*H0;
		P = 101325.0*exp((log(288.15 - 0.0065*H0) - log(288.15))/R/0.0065);
     else if(H0>11000 && H0<=20000)	
             T = 216.65;                                                       
             P = 22632.1*exp(-(H0-11000.0)/R/216.65);      
         else if(H0>20000 && H0<=32000)   
                 T = 196 + 0.001*H0;		
                 P = 5474.906*exp(-(log(196.65 + 0.001*H0) - log(216.65))/R/0.001);
             else if(H0>32000 && H0<=47000)
                     T = 139.05+0.0028*H0;                                                 
                     P = 868.0238*exp(-(log(139.05+0.0028*H0)-log(228.65))/R/0.0028);                             
                 else if(H0>47000 && H0<=51000)
                         T = 270.65;                                                       
                         P = 110.9072*exp(-(H0-47000.)/R/270.65);
                     else if(H0>51000 && H0<=71000)		
                             T = 413.45-0.0028*H0;                                 
                             P = 66.93945*exp((log(413.45-0.0028*H0)-log(270.65))/R/0.0028);
                         else if(H0>71000 && H0<=84852)
                                 T = 356.65-0.002*H0;                                        
                                 P = 3.956471*exp((log(356.65-0.002*H0)-log(214.65))/R/0.002);
                             else
                                 T = 186.8673;
                                 P = 0.3733889*exp(-(H0 - 84852)/R/186.8673);
                             end
                             
                         end
                         
                     end
                     
                 end
                 
             end
             
         end
    end
    
	rho = 1.225*288.15*P/T/101325;
	Ma_ = 20.0468*sqrt(T);

end

