import math
from OtterDynamicClass import Otter
from colreg_define import  Colreg_Define
from dynamic_obs import TargetVehicle
from mpccontroller import MPCUSV
from mpc_colav import MPCUSV_colav
import pandas as pd
from path_track_2 import LineofSight
import casadi as ca 
import numpy as np 



obstacle = TargetVehicle(30,10,90,1.2)
OtterPredict = TargetVehicle(0,0,90,0)
colreg_detect = Colreg_Define()
los = LineofSight()




def set_obstacle(diam_usv=2,diam_obs=3):
    """set_obstacle fonksiyonu dinamik engel oluştrumak için kullanilir. (X,Y,heading,velocity) parametrelerini alabilir."""


    obstacle.KinematicTarget()   # target object için 
    obstacle.prediction_movement() # target object için 
    
    dist = math.sqrt((OtterPredict.current_eta[0]-obstacle.x)**2+(OtterPredict.current_eta[1]-obstacle.y)**2)-(diam_usv/2+diam_obs/2+0.5) 
    safe_dist = 25

    if dist>safe_dist:
        path_follow=True

    else:
        path_follow=False

    output = dict(path_follow=path_follow,obs_x=obstacle.x,obs_y=obstacle.y,safe_dict= safe_dist,obs_heading=obstacle.course,obs_pre_x=obstacle.predict_x_list,obs_pre_y=obstacle.predict_y_list,
                obs_pre_heading=obstacle.predict_theta_list,obs_speed=obstacle.U_speed,obs_theta_dot = obstacle.theta_dot)
    # return path_follow, obstacle.x,obstacle.y,safe_dist,obstacle.course,obstacle.predict_x_list,obstacle.predict_y_list,obstacle.predict_theta_list,obstacle.U_speed,obstacle.theta_dot
    return output


Wpx = [10,45,60,80]
Wpy = [20,30,70,30]

init_states = [0,0,0,0,0,0]
coeff = los.path_generate(Wpx,Wpy)
OtterPredict.current_eta=np.array([10,20,0,0,0,math.radians(30)])

for i in range(5000):
    OtterPredict.prediction_movement()
    los_output = los.execute(OtterPredict.nu,OtterPredict.current_eta,Wpx,Wpy)
    set_output = set_obstacle()
    
    mpc_los = MPCUSV()
    mpc_collosion = MPCUSV_colav(set_output['obs_heading'],set_output['obs_x'],set_output['obs_y'])

    ship1 = pd.DataFrame({'MMSI':[1], 'LAT':OtterPredict.current_eta[0], 'LON':OtterPredict.current_eta[1], 'SOG':math.sqrt(OtterPredict.nu[0]**2+OtterPredict.nu[1]**2), 'COG':math.degrees(OtterPredict.current_eta[5])})
    ship2 = pd.DataFrame ({'MMSI':[1], 'LAT':set_output['obs_x'], 'LON':set_output['obs_y'], 'SOG':obstacle.v, 'COG':set_output['obs_heading']})

  
    colreg = colreg_detect.COLREG_detect(ship1,ship2)
    colreg_detect.colreg_situation(colreg,OtterPredict.predict_x_list,OtterPredict.predict_y_list,set_output['obs_speed'],set_output['obs_x'],set_output['obs_y'],set_output['obs_heading'],set_output['obs_pre_x'],
                                   set_output['obs_pre_y'],set_output['obs_pre_heading'],set_output['obs_theta_dot'])

    colreg,mpc_output=colreg_detect.colreg_execute(set_output['path_follow'],init_states,los_output,set_output,colreg)

    u_d = np.array(mpc_output['control_signal'].full())
    OtterPredict.u_control = [float(u_d[0][0]),float(u_d[1][0])]
    output_func=OtterPredict.function()
    print('Otter Predict Function',output_func)

    x_init=output_func['current_eta'][0]
    y_init=output_func['current_eta'][1]
    yaw_init=output_func['current_eta'][5]
    init_states = ca.vertcat(0,0,0,x_init,y_init,yaw_init)
    

    OS_Arr =[OtterPredict.predict_x_list,OtterPredict.predict_y_list,OtterPredict.predict_theta_list]
    TS_Arr = [set_output['obs_pre_x'],set_output['obs_pre_y'],set_output['obs_pre_heading']]

    los.los_simulation(OtterPredict.current_eta,Wpx,Wpy,OtterPredict.u_control,set_output['obs_x'],set_output['obs_y'],[colreg_detect.col_dist*math.cos(math.radians(set_output['obs_heading']+180)+colreg_detect.col_dist*math.sin(math.radians(set_output['obs_heading']+180))),colreg,],OS_Arr,TS_Arr)
    