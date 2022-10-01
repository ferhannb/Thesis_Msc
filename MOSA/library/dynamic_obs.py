import numpy as np
from gnc import attitudeEuler
import math
import matplotlib.pyplot as plt
from OtterDynamicClass import Otter

class TargetVehicle(Otter):
    """Bu classda çevrediki dinamik engellerin ve kendi aracimizin rotalarini tahmin eden fonksiyonlar yer almaktadir."""

    def __init__(self,x=35,y=15,theta=90,v=3,omega=0,controlSystem="stepInput", 
                r = 0, 
                V_current = 0, 
                beta_current = 0, 
                tau_X = 120):

        super().__init__(controlSystem,r,V_current,beta_current,tau_X)
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        self.predict_x = 0
        self.predict_y = 0
        self.x_list=[]
        self.y_list=[]
        self.theta_list=[]
        self.safe_dist = 25
        self.course = 0
        

        
    

            

    def KinematicTarget(self,dt=0.02):
        """Arac hareket eden nesneler için kinematik denklem"""

        A = np.array([[math.cos(math.radians(self.theta)),0],[math.sin(math.radians(self.theta)),0],[0,1]])
        B = np.array([[self.v],[self.omega]])
        x_dot = A@B
        self.x_next = self.x + dt*x_dot[0]
        self.y_next = self.y + dt*x_dot[1]
        self.theta = self.theta + dt*x_dot[2]
        self.course =math.degrees(math.atan2(self.y_next-self.y,self.x_next-self.x))
        self.course = [(self.course + 360) % 360]
        self.x = self.x_next
        self.y = self.y_next

        self.x=self.x[0]
        self.y=self.y[0]
        self.course=self.course[0] 
        self.U_speed = math.sqrt(x_dot[0]**2+x_dot[1]*2) # vehicle Course speed 

    

    def prediction_movement(self,dt=0.02):
        """Geçmiş seyir verilerinden yararlanarak aracin geşecekteki konum verilerinin çikarilmasi"""


        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.theta_list.append(self.course)
   

        if len(self.x_list)==101:
            self.x_list.remove(self.x_list[0])
            self.y_list.remove(self.y_list[0])
            self.theta_list.remove(self.theta_list[0])
            

        x_dot_list = np.diff(self.x_list)
        len_x = len(x_dot_list)
        if len(x_dot_list)==0:
            len_x=1
        self.x_dot = (sum(x_dot_list)/(len_x))/dt

        y_dot_list = np.diff(self.y_list)
        len_y = len(y_dot_list)
        if len(y_dot_list)==0:
            len_y = 1
        self.y_dot = (sum(y_dot_list)/(len_y))/dt


        theta_dot_list = np.diff(self.theta_list)
        theta_len = len(theta_dot_list)
        if len(theta_dot_list)==0:
            theta_len = 1
        self.theta_dot = (sum(theta_dot_list)/(theta_len))/dt
        print('theta dot',self.theta_dot)


        self.predict_x_list=[]
        self.predict_y_list=[]
        self.predict_theta_list=[]



        U = math.sqrt(self.x_dot**2+self.y_dot**2)
        x_ =self.x
        y_ =self.y
        theta_dot=self.theta_dot

        theta_ =self.course

        for _ in range(1000):
            
            A = np.array([[math.cos(math.radians(theta_)),0],[math.sin(math.radians(theta_)),0],[0,1]])
            B = np.array([[U],[theta_dot]])
            x_dot = A@B

            x_next = x_ + dt*x_dot[0]
            y_next = y_ + dt*x_dot[1]
            theta_next= theta_ + dt*x_dot[2]
            theta_ =math.degrees(math.atan2(y_next-y_,x_next-x_))
            theta_ = [(theta_ + 360) % 360]

            x_ = x_next
            y_ = y_next
            theta_ = theta_next

            self.predict_x_list.append(x_next ) # predicted x of vehicle 
            self.predict_y_list.append(y_next)  # predicted y of vehicle 
            self.predict_theta_list.append(theta_) # predicted theta of vehicle 
    
    def set_obstacle(self,obs_x,obs_y,obs_theta,obs_v,diam_usv=2,diam_obs=3):
        "Dinamik engellin konum ve heading tahmini"
        obstacle = TargetVehicle(obs_x,obs_y,obs_theta,obs_v)
        obstacle.KinematicTarget()   
        x_obsArr,y_obsArr,courseArr,TS_theta_dot=self.prediction_movement()
        dist = math.sqrt((self.current_eta[0]-self.x)**2+(self.current_eta[1]-self.y)**2)-(diam_usv/2+diam_obs/2+0.5) 
        
        if dist>self.safe_dist:
            param=True
        else:
            param=False
        return param,x_obsArr,y_obsArr,courseArr,TS_theta_dot

        


    def simulation(self,predict_x_list,predict_y_list,x_o,y_o):

        plt.clf()
        plt.xlim([0,120])
        plt.ylim([0,120])
        plt.gcf().gca().add_artist(plt.Circle((self.x,self.y),2,fill=False))
        plt.plot(predict_x_list,predict_y_list,'r.-')
        plt.plot(x_o,y_o,'m.-')
        plt.show(block=False)
        plt.pause(0.02)




if __name__=='__main__':
    
    target = TargetVehicle()
    print(target.current_eta)
    

