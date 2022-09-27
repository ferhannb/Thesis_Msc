#!/usr/bin/env python
# -*- coding: utf-8 -*-



from USVController import USVController
from OtterDynamicClass import Otter
from path_track_2 import LineofSight
from path_track_3 import LineofSightfirst
import matplotlib.pyplot as plt 
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly
import csv
import numpy as np 

pid = USVController()
vehicle = Otter()
path_generation = LineofSight()
path_generation_diff = LineofSightfirst()




vehicle.initialize_otter()

x_list = []
y_list = []
speed_list = []
speed_ref_list = []
heading_list =  []
sancak_rpm_signal = []
iskele_rpm_signal = []
sancak_rpm = []
iskele_rpm = []
time_list = []
speed_error=[]
heading_ref_list = []
speed_error_list = [] 
heading_error_list = []  
csv_velocity_list = []
csv_time_list = []
U_p_list = []
U_i_list = []
U_total_list = []
time_=0
Kp_heading = -10
Ki_heading = -4
Kd_heading = 0
Kp_speed = 20#13 velocity PID için 10 positional PID için 20
Ki_speed = 0.08#0.1 velocity PID içib 20 positional PID için 0.08
Kd_speed = 0

for _ in range(3000):
    time_+=0.02

    ### REFERANS VALUES FOR SPEED AND HEADING
    ref_heading = 0 
    if _  <500:
        ref_speed = 3.08
    elif _ <800:
        ref_speed = 1
    elif _<1500:
        ref_speed = 2.2
    elif _<2500:
        ref_speed = 0.5
    elif _<3000:
        ref_speed=1.75
 
    speed_otter = math.sqrt(vehicle.nu[0]**2+vehicle.nu[1]**2)
    filtred_heading_signal = pid.Filtred_heading_referans(ref_heading)
    filtred_speed_signal = pid.Filtred_speed_signal(ref_speed)
    U_diff = pid.Heading_controller(filtred_heading_signal,vehicle.current_eta,Kp_heading,Ki_heading,Kd_heading)
    u_avg = pid.Speed_controller(filtred_speed_signal,speed_otter,Kp_speed,Ki_speed,Kd_speed,pid_method=1)
    u_avg = pid.reset_integral(u_avg)
    U_diff = pid.reset_integral_heading(U_diff)
    vehicle.u_control = pid.control_allocation(u_avg,U_diff)

    # if _<1000:
    #     vehicle.u_control=[104,104]
    # elif _<2500:
    #     vehicle.u_control=[90,90]
    # elif _<4000:
    #     vehicle.u_control=[80,80]
    # elif _<5500:
    #     vehicle.u_control=[70,70]
    # elif _<7000:
    #     vehicle.u_control=[60,60]
    # elif _<9500:
    #     vehicle.u_control=[50,50]
    # elif _<11000:
    #     vehicle.u_control=[40,40]
    # elif _<13500:
    #     vehicle.u_control=[30,30]
    # elif _<15000:
    #     vehicle.u_control=[20,20]
    # else:
    #     vehicle.u_control=[10,10] 
    # if _<1000:
    #     vehicle.u_control=[104,104]
    # if _<2000:
    #     vehicle.u_control=[104,104]
    # elif _<3000:
    #     vehicle.u_control=[50,50]   
    output = vehicle.function()
    # print(output['heading'])

    if _>500 and _<800:
        csv_velocity_list.append(output['speed'])
        csv_time_list.append(time_)

    speed_ref_list.append(filtred_speed_signal)
    heading_ref_list.append(filtred_heading_signal)
    speed_list.append(output['speed'])
    time_list.append(time_ )
    x_list.append(vehicle.current_eta[0])
    y_list.append(vehicle.current_eta[1])
    heading_list.append(vehicle.current_eta[-1])
    sancak_rpm_signal.append(vehicle.u_control[0])
    iskele_rpm_signal.append(vehicle.u_control[1])
    iskele_rpm.append(vehicle.u_actual[1])
    sancak_rpm.append(vehicle.u_actual[0])
    speed_error_list.append(pid.speed_error)    
    heading_error_list.append(pid.heading_error)
    U_total_list.append(u_avg)
    U_i_list.append(pid.U_i)
    U_p_list.append(pid.U_p)


np.savetxt("incremetnal_speed.csv",np.column_stack((csv_time_list,csv_velocity_list)),delimiter=",",fmt='%s')
fig=go.Figure()

fig = make_subplots(rows=4, cols=1)
# Add traces
fig.add_trace(go.Scatter(x=time_list, y=speed_list,
                    mode='lines',
                    name='araç hiz'),row=1, col=1)
fig.add_trace(go.Scatter(x=time_list, y=speed_ref_list,
                    mode='lines',
                    name='cmd hiz'),row=1, col=1)



fig.add_trace(go.Scatter(x=time_list, y=speed_error_list,
                    mode='lines',
                    name='speed error'),row=1, col=1)

fig.add_trace(
    go.Scatter(x=time_list, y=sancak_rpm,name=' Sancak pervane'),row=2, col=1
    )

fig.add_trace(
    go.Scatter(x=time_list, y=iskele_rpm,name='İskele Pervane'),
    row=2, col=1)

fig.add_trace(
    go.Scatter(x=time_list, y=sancak_rpm_signal,name='control signali_sancak'),
    row=2, col=1)

fig.add_trace(
    go.Scatter(x=time_list, y=iskele_rpm_signal,name='control signali_iskele'),
    row=2, col=1)

fig.add_trace(
    go.Scatter(x=time_list, y=U_total_list,name='Total U'),
    row=3, col=1)

fig.add_trace(
    go.Scatter(x=time_list, y=U_p_list,name='U_p'),
    row=3, col=1)

fig.add_trace(
go.Scatter(x=time_list, y=U_i_list,name='U_i'),
row=3, col=1)

fig.add_trace(
    go.Scatter(x=x_list, y=y_list,name='x-y pervane'),row=4, col=1
    )
fig.update_layout( title_text="Sinus referansi")
plotly.offline.plot(fig, filename="Kare-2.html")

fig.show()

