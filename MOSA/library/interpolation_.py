import numpy as np
import pandas as pd 
import matplotlib.pyplot as plt


class OtterPolation():

    def __init__(self):
        self.positif_velocity=[0,0.02,0.06,0.11,0.17,0.25,0.35,0.45,0.57,0.714,0.86,1.02,1.2,1.4,1.6,1.82,2.06,2.31,2.57,3.08]
        self.positif_rps = [0,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,104]
        self.negative_velocity = np.flip([0,-0.01,-0.068,-0.145,-0.27,-0.41,-0.59,-0.81,-1.06,-1.34,-1.71])
        self.negative_rps = np.flip([0,-10,-20,-30,-40,-50,-60,-70,-80,-90,-104])
        
    def interpolation(self,ref_speed):
        if ref_speed>=0:
            value = np.interp(ref_speed,self.positif_velocity,self.positif_rps)
        else:
            value = np.interp(ref_speed,self.negative_velocity,self.negative_rps)
        return value 

if __name__=='__main__':
    a = OtterPolation()
    ref_speed=-0.1
    rps_ = a.interpolation(ref_speed)
    
    plt.plot(a.negative_velocity,a.negative_rps,'m*')
    plt.plot(ref_speed,rps_,'ro')
    plt.show()

