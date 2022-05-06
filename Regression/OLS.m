clear all

fileID = fopen('v_bag.txt','r');
formatSpec = '%s';
bag=fscanf(fileID,formatSpec)
fclose(fileID);

v_b=sscanf(bag,'vbag:%f---');


fileID = fopen('v_computed.txt','r');
formatSpec = '%s';
computed=fscanf(fileID,formatSpec);
fclose(fileID);

v_c=sscanf(computed,'vcomputed:%f---');


lc=length(v_c);
lb=length(v_b);
j=1;
v_c_=zeros(min(lc,lb),1);
v_b_=v_c_;

%%ALIGN VECTORS TO AVOID EXTRA DATA AND SYNC.

if lc>lb
      for i=1:lb
      v_c_(i)=v_c(i);
      end
      v_b_= v_b;
        
    else if lb>lc 
             for i=1:lc
                v_b_(i)=v_b(i);
             end
                v_c_=v_c;        
        else if lb==lc
               v_c_=v_c;
               v_b_=v_b;
            
            end   
     end
end


                      
                    
                    
                
     



w=inv(v_c_'*v_c_)*v_c_'*v_b_;




figure
scatter(v_b_,v_c_)
hold on
v_b1=w*v_c_;
plot(v_b1,v_c_);

