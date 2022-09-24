import numpy as np
from scipy.interpolate import interp2d
import matplotlib.pyplot as plt




# f = interp2d(hizArr,sapmaArr,r,kind='linear',fill_value='-1')


def speed_generate(radius,sapma):



    hizArr = np.array([0,10,20,30,40,50,60,70,80,90,100])
    sapmaArr = np.array([0,10,20,30,40,50,60,70,80,90,100])

    r = np.array([(0,0,0,0,0,0,0,0,0,0,0),(0.42,1.86,2.85,4.5,6.5,8.9,11.7,14.91,18.55,22.64,41.36),(0.55,2.05,2.85,3.6,4.71,6.1,7.76,9.68,11.85,15.6,22.64),
    (0.69,2.17,3.2,3.84	,4.53,5.48,6.66	,8.05,9.55,	11.85,15.6),(0.83,2.25,3.84,4.27,4.89,5.54,6.42,7.06,8.05,9.55,11.85),(0.97,2.35,3.4,4.56,5.34,5.95,6.07,6.42,7.06,8.05,9.55),
    (1.11,2.45,3.7,4.79,5.69,5.89,5.95,6.07,6.42,7.06,8.05),(1.24,2.55,4.75,4.98,5.39,5.69,5.89,5.95,6.07,6.42,7.06),(1.36,2.66,3.94,4.49,4.98,5.39,5.69,5.89,5.95,6.07,6.42),
    (1.5,2.76,3.35,3.94,4.49,4.98,5.39,5.69,5.89,5.95,6.07),(1.6,2.17,2.76,3.35,3.94,4.49,4.98,5.39,5.69,5.89,5.95)])


    if radius>41.36:
        radius=41.35
    print('radiussss',radius)
    diff_sapma = np.absolute(sapmaArr-sapma)
    idx_sapma = diff_sapma.argmin()
    print(np.max(r[idx_sapma,:]))

    if np.max(r[idx_sapma,:])<radius:
        print('********')
        print(max(r[idx_sapma,:]))
        while (np.max(r[idx_sapma,:])<=radius):
            # print('-----')
            # print(idx_sapma)
            idx_sapma -=1

            # if idx_sapma <0:
            #     idx_sapma =0
        

    print('idx_sapma:',idx_sapma)
    selected_sapma_row = r[idx_sapma,:]
    print(selected_sapma_row)
    diff_arr = np.absolute(selected_sapma_row-radius)
    idx = diff_arr.argmin()

    if radius>41.36:
        assign_v_avg=100

    if selected_sapma_row[idx]<radius:

        if idx<=10:
            
            delta_r = selected_sapma_row[idx+1]-selected_sapma_row[idx]

        
        assign_v_avg = 10*(radius-selected_sapma_row[idx])/delta_r+hizArr[idx]  

    elif selected_sapma_row[idx]>radius: 
        delta_r = selected_sapma_row[idx]-selected_sapma_row[idx-1]
        assign_v_avg = 10*(radius-selected_sapma_row[idx-1])/delta_r+hizArr[idx-1]
        if idx<0:
            idx=0
    else:
        assign_v_avg = float(hizArr[idx])
    print(assign_v_avg)
    v_signal = 0.000285*assign_v_avg*assign_v_avg
    return v_signal



    
## Radius 3d grafik cıkarımı.(OK)
## dinamik engelin kinematiğini eklemek.
## uğur şimsir. makale
## rad_cur ile path çıkar path takip et.




   


    


    



