
from OtterDynamicClass import Otter
from path_track_2 import LineofSight
from path_track_3 import LineofSightfirst
from USVController import USVController

import matplotlib.pyplot as plt 
import math
import numpy as np
from path_generation.CubicCurve import generate_curve

vehicle = Otter()
los_index = LineofSight()
los_numeric = LineofSightfirst()
pid = USVController()

vehicle.initialize_otter()
vehicle.current_eta=np.array([10,10,0,0,0,math.radians(50)])
# Wpx = [10,20,30,40]
# Wpy = [10,20,10,20]
Wpx = [10,30,50,70,90]
Wpy = [10,20,10,20,10]
# Wpx = [10,30,50,70]
# Wpy = [10,20,10,40]
### non-index method
# Wpx = [10,20,30,40]
# Wpy = [10,10,10,10] 
#### PID KATSAYILARI #######
### Speed ###

Kp_speed = 20
Ki_speed = 0.08
Kd_speed = 0

### Heading ###

Kp_heading=-4.5
Ki_heading=-0.0000
Kd_heading=-20



v_list = []
head_list=[]
x_list = []
y_list = []
eta_list = []
ref_speed = []
u_control_sancak = []
u_control_iskele = []
x_closest_list = []
y_closest_list = []
x_los_list = []
y_los_list = []
pervane_iskele = []
pervane_sancak = []
chi_d_list = []
actualheading_list = []
timeOtter=[]
refheading_list = []

los_index.path_generate(Wpx,Wpy) # self.coeff ,self.x_init, self.y_init,self.Wp_x_init,self.Wp_y_init
Time = 0.0
e,a,b,c,d = generate_curve(Wpx,Wpy,0.01)
for i in range(3500):

    Time+=0.02
    timeOtter.append(Time)

    los_output=los_index.execute(vehicle.nu,vehicle.current_eta,Wpx,Wpy)
    referans_U = los_output['U_desired']
    refChi=math.degrees(los_output['chi_d'])%360

    

    speed_otter = math.sqrt(vehicle.nu[0]**2+vehicle.nu[1]**2)
    filtred_heading_signal = pid.Filtred_heading_referans(refChi)
    filtred_speed_signal = pid.Filtred_speed_signal(referans_U)
    U_diff = pid.Heading_controller(filtred_heading_signal,vehicle.current_eta,Kp_heading,Ki_heading,Kd_heading)
    u_avg = pid.Speed_controller(filtred_speed_signal,speed_otter,Kp_speed,Ki_speed,Kd_speed,pid_method=1)
    u_avg = pid.reset_integral(u_avg)
    U_diff = pid.reset_integral_heading(U_diff)
    
    vehicle.u_control = pid.control_allocation(u_avg,U_diff)
    # if i <750:
    #     vehicle.u_control = [100,100]
    # else:
    #     vehicle.u_control = [-100,100]
    output = vehicle.function()
    
    #### FOR PLOTTING ####
    v_list.append(np.sqrt(vehicle.nu[0]**2+vehicle.nu[1]**2))  # for plotting
    head_list.append(vehicle.current_eta[5]*180/math.pi %360) # for plotting
    x_list.append(vehicle.current_eta[0]) # for plotting 
    y_list.append(vehicle.current_eta[1]) # for plotting 
    eta_list.append(vehicle.current_eta)
    ref_speed.append(filtred_speed_signal)
    u_control_sancak.append(vehicle.u_control[0])
    u_control_iskele.append(vehicle.u_control[1])
    x_closest_list.append(los_index.x_closest)
    y_closest_list.append(los_index.y_closest)
    x_los_list.append(los_index.x_los)
    y_los_list.append(los_index.y_los)
    pervane_iskele.append(vehicle.u_actual[1])
    pervane_sancak.append(vehicle.u_actual[0])
    chi_d_list.append(refChi)
    refheading_list.append(filtred_heading_signal)
    actualheading_list.append(vehicle.current_eta[5]*180/math.pi %360)

    


    # los_index.los_simulation(vehicle.current_eta,Wpx,Wpy,vehicle.u_control)


    if los_index.x_los==los_index.x_closest and los_index.y_los == los_index.y_closest:

        break 
    
        

    # ani=los2.animate(x_list,y_list,head_list,Wpx,Wpy,x_closest_list,y_closest_list,x_los_list,y_los_list,u_control,i)
    # figure, ax = plt.subplots()
    # animation = FuncAnimation(figure,
    #                       func = ani,
    #                       frames = np.arange(0, 10, 0.1), 
    #                       interval = 10)
    # plt.show()
        
print(u_control_sancak)        
print('yyyyyyyyyy')  
plt.figure()
plt.plot(x_list,y_list)
plt.plot(a,b)
plt.xlabel('X')
plt.ylabel('Y')
plt.figure()
plt.plot(timeOtter,v_list)
plt.plot(timeOtter,ref_speed)
plt.xlabel('Zaman')
plt.ylabel('HIZ')
plt.figure()
plt.plot(timeOtter,u_control_sancak)
plt.plot(timeOtter,pervane_sancak)
plt.title('Sancak')
plt.figure()
plt.plot(timeOtter,u_control_iskele)
plt.plot(timeOtter,pervane_iskele)
plt.title('İskele')

plt.figure()
plt.plot(timeOtter,chi_d_list,'r')
plt.plot(timeOtter,actualheading_list)
plt.title('Heading')


plt.figure()
plt.plot(timeOtter,refheading_list,)
plt.title('filtred ref signal')

plt.xlabel('Zaman')
plt.ylabel('Kontrol Sinyali')
plt.show()    












