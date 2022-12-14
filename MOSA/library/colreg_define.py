
import math
from dynamic_obs import TargetVehicle
from path_track_2 import LineofSight
from mpccontroller import MPCUSV
from mpc_colav import MPCUSV_colav

class Colreg_Define(TargetVehicle):

    def __init__(self):
        super().__init__()
        self.colreg='Safe'
        self.los = LineofSight()
        self.mpc_los = MPCUSV()
        self.mpc_collision = MPCUSV_colav(0,0,0)
        self.col_dist = 5 

    def convert_east_north(self,radian): 
        """From vessel coordinate frame to COLREG coordinate frame"""
        
        if radian < 0:
            radian =  math.pi/2 + abs(radian)
        elif radian >= 0 and radian < math.pi/2:
            radian = math.pi/2 - radian
        else:
            radian = -(radian-math.pi/2) + math.pi * 2
        
        return math.degrees(radian) % 360

    def COLREG_detect(self,ship1, ship2):
        y1, x1, v1, c1 = ship1.LAT, ship1.LON, ship1.SOG, ship1.COG
        y2, x2, v2, c2 = ship2.LAT, ship2.LON, ship2.SOG, ship2.COG
        c1degree = c1
        c2degree = c2
        c1 = c1 / 180 * math.pi # Convert to radians
        c2 = c2 / 180 * math.pi
        distance_between = 0


        k2 = math.cos(c1)**2 * v1**2 \
            - 2  *  v1 * v2 * math.cos(c1)* math.cos(c2) \
            + math.cos(c2)**2 * v2**2 \
            + math.sin(c1)**2 * v1**2 \
            - 2  * v1  * v2 * math.sin(c1) * math.sin(c2)\
            + math.sin(c2)**2 * v2**2 
        
        k1 = 2 * math.sin(c1) * v1 * x1 \
                - 2 * v1 * x2 * math.sin(c1) \
                + 2 * v1 * y1 * math.cos(c1) \
                - 2 * v1 * y2 * math.cos(c1) \
                - 2 * v2 * x1 * math.sin(c2) \
                + 2 * v2 * x2 * math.sin(c2) \
                - 2 * v2 * y1 * math.cos(c2) \
                + 2 * v2 * y2 * math.cos(c2)

        k0 = x1**2 + x2**2 + y1**2 + y2**2 - 2 * x1 * x2 - 2 * y1 * y2
            
        xt = -k1 / (2 * k2)
        xt_ = xt
        
        CPA = math.sqrt(abs(k2*xt**2 + k1*xt + k0))
        
        if xt.values < 0:
            xt_ = 0
        elif c1.values == c2.values:
            xt_ = 0
        else:
            xt_ = xt[0]

        xx1 = x1 + math.sin(c1) * v1 * 0.02
        yy1 = y1 + math.cos(c1) * v1 * 0.02
        
        xx2 = x2 + math.sin(c2) * v2 * 0.02
        yy2 = y2 + math.cos(c2) * v2 * 0.02
        
        dy = yy2 - yy1
        dx = xx2 - xx1
        
    
        AbsTarget = math.atan2(dy,dx) 
        AbsTarget = self.convert_east_north(AbsTarget)
        RelTarget = (-AbsTarget + c1degree) % 360 # Relative bearing of target ship at the time of CPA from ownship. Always Clockwise and positive.
        
        AbsOwn = (AbsTarget-180) % 360 # Absolute bearing of ownship at the time of CPA from target ship.
        RelOwn = (-AbsOwn + c2degree) % 360  # Relative bearing of ownship at the time of CPA from target ship.

        #### CPA RELEATING BEARINGS ###

        xx1_cpa = x1 + math.sin(c1) * v1 * xt_
        yy1_cpa = y1 + math.cos(c1) * v1 * xt_
        
        xx2_cpa = x2 + math.sin(c2) * v2 * xt_
        yy2_cpa = y2 + math.cos(c2) * v2 * xt_

        dy_cpa = yy2_cpa - yy1_cpa
        dx_cpa = xx2_cpa - xx1_cpa

        AbsTarget_cpa = math.atan2(dy_cpa,dx_cpa) 
        AbsTarget_cpa = self.convert_east_north(AbsTarget_cpa)
        RelTarget_cpa = (-AbsTarget_cpa + c1degree) % 360 # Relative bearing of target ship at the time of CPA from ownship. Always Clockwise and positive.
        
        AbsOwn_cpa = (AbsTarget_cpa-180) % 360 # Absolute bearing of ownship at the time of CPA from target ship.
        RelOwn_cpa = (-AbsOwn + c2degree) % 360



        if self.colreg=='Safe':
            if (-12%360) <=RelTarget[0] and RelTarget[0]<=360 or  0<=RelTarget[0] and RelTarget[0]<=12 and (-12%360) <=RelOwn[0] and RelOwn[0]<=360 or  0<=RelOwn[0] and RelOwn[0]<=12:
                self.colreg = 'Head-on'           
            elif 12<RelTarget[0] and RelTarget[0]<=112.5 and 247.5<RelOwn[0] and RelOwn[0] <354:            
                colreg = 'Give-Away'
        elif self.colreg =='Give-Away':
            if 270<=RelTarget[0]  and RelTarget[0]<=300:
                self.colreg='Safe'
        elif self.colreg =='Head-on':
            if RelTarget[0]>=260 and RelTarget[0]<=280:
                self.colreg='Safe'


        
                
    
        Output = dict(COLREG=self.colreg,Distance_between=distance_between,RelOwn=RelOwn[0],RelTarget=RelTarget[0])
        # return xt_, CPA, distance_between, AbsTarget, RelTarget[0], AbsOwn, RelOwn[0]
        return Output


    


    def colreg_situation(self,colreg,x_oArr,y_oArr,Ts_speed,TS_x,TS_y,TS_heading,x_tArr,y_tArr,c_tArr,theta_dot):

        """Bu fonksiyonda OS ve TS'in COLREG durumuna bakilir. OS current_eta, prediction list TS kinematik func. verileri ve prediction list 
            verilerine g??re ??arp????ma durumlar?? kontrol edilir."""


        OS_x =  self.current_eta[0]
        OS_y = self.current_eta[1]
        OS_heading = self.current_eta[5]
        U_speed = math.sqrt(self.nu[0]**2+self.nu[1]**2)
        

        x_dist = ([abs(a_x-b_x) for a_x, b_x in zip(x_oArr,x_tArr)])
        min_value = min(x_dist)
        min_index = x_dist.index(min_value)


        joint_x = x_oArr[min_index]  ### Kesi??im noktas??s??n??n X de??eri
        joint_y = y_oArr[min_index]  ### KEsi??im noktas??n??n??n Y de??eri

        OS2Joint = math.sqrt((OS_x-joint_x)**2+(OS_y-joint_y)**2) ### OS ile kesi??im noktas?? aras?? uzakl??k
        print('OS2Joint',OS2Joint)
        TS2Joint = math.sqrt((TS_x-joint_x)**2+(TS_y-joint_y)**2) ### TS ile kesi??im noktas?? aras?? uzakl??k
        print('OS2Joint',TS2Joint)
        if U_speed==0:
            U_speed=0.1

        OS_int_time = OS2Joint/U_speed # X/V
        print('OS_int_time',OS_int_time)
        print('Ts_speed',Ts_speed)
        if Ts_speed ==0:
            Ts_speed=0.01
        TS_int_time = TS2Joint/Ts_speed #X/V
        Safe_time = 5 
        ### Kesi??im noktalar??na ula??ma s??resini belirler.
        y_dist=[abs(a_y-b_y) for a_y, b_y in zip(y_oArr,y_tArr)] 
        min_x =min(x_dist)
        min_y =min(y_dist)
        mindistV2V=math.sqrt(min_x**2+min_y**2)
        print('MIN DISTANCE',mindistV2V)
        # Yukar??daki fonksiyon predict edilen ara?? do??rultular??n??n kesi??im durumlar??na bakar.
        
        
        if colreg['COLREG']=='Give-Away':

            if mindistV2V<0.5: # Rotalar Kesi??iyorsa
                print('KES??????M')
                colreg['COLREG'] = 'Give-Away'
                if OS_int_time>TS_int_time+5 : #or OS_int_time<TS_int_time+5
                    print('----------------------------------')
                    colreg['COLREG'] = 'Safe'
                if  OS_int_time+5<TS_int_time : #or OS_int_time<TS_int_time+5
                    print('**********************************')
                    colreg['COLREG'] = 'Safe'

            elif theta_dot !=0 and math.sqrt((x_oArr[-1]-x_tArr[-1])**2+(y_oArr[-1]-y_tArr[-1])**2)>5:

                colreg['COLREG'] = 'Safe'              

        return colreg

    def colreg_execute(self,path_follow,init_states,los_output,output_set,colreg):

        if path_follow:
            mpc_output = self.mpc_los.execute_MPC(init_states,[los_output['U_desired'],0,0,los_output['x_los'],los_output['y_los'],los_output['chi_d']])
        else:
            
            print('==OBSTACLE')
            if colreg['COLREG']=='Head-on':
                # print('HEAD_ON')
                sc='HEAD-ON'
                mpc_output= self.mpc_collision.execute_MPC(init_states,[los_output['U_desired'],0,0,los_output['Wpx'],los_output['Wpy'],math.atan2((los_output['Wpy']-self.current_eta[1]),(los_output['Wpx']-self.current_eta[0]))])
                if 180<=colreg['RelTarget'] and colreg['RelTarget']<=270:
                    mpc_output= self.mpc_los.execute_MPC(init_states,[0,0,0,los_output['x_los'],los_output['y_los'],los_output['chi_d']])

            elif colreg['COLREG']=='Give-Away':
                # print('Give-Away')
                mpc_output= self.mpc_collision.execute_MPC(init_states,[los_output['U_desired'],0,0,output_set['obs_x']+self.col_dist*math.cos(math.radians(output_set['obs_heading']+180))
                            ,output_set['obs_y']+self.col_dist*math.sin(math.radians(output_set['obs_heading']+180)),math.atan2((output_set['obs_y']-self.current_eta[1]),(output_set['obs_x']+10-self.current_eta[0]))])
                # mpc_output= mpc.execute_MPC(init_states,[0,0,0,x_obs+10,y_obs,math.atan2((y_obs-current_eta[1]),(x_obs+10-current_eta[0]))])
                # mpc_output= mpc.execute_MPC(init_states,[0,0,0,los_output['Wpx'],los_output['Wpy'],math.atan2((los_output['Wpy']-current_eta[1]),(los_output['Wpx']-current_eta[0]))])
            elif colreg['COLREG']=='Safe':
                # print('11Safe')
                mpc_output= self.mpc_los.execute_MPC(init_states,[los_output['U_desired'],0,0,los_output['x_los'],los_output['y_los'],los_output['chi_d']])
        
        return colreg,mpc_output



    
# if __name__=='__main__':

deneme = Colreg_Define()
print(deneme.current_eta)

#     colreg_ = Colreg_Define()

#     print(TargetVehicle.current_eta)

#     ship1 = pd.DataFrame({'MMSI':[1], 'LAT':[0], 'LON':[0], 'SOG':[10], 'COG':[90]})
#     ship2 = pd.DataFrame({'MMSI':[2], 'LAT':[2000], 'LON':[2000], 'SOG':[10], 'COG':[180]})
    
#     output = colreg_.COLREG_detect(ship1,ship2)

#     col = colreg_.colreg_situation()


    




