
from email.utils import collapse_rfc2231_value
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as ph
from path_generation.CubicCurve import generate_curve
from path_generation.time_depent_curve import spline_gen
from R_calculation import R_Calculator
from matplotlib.animation import FuncAnimation
from itertools import chain
from scipy.optimize import minimize
import sympy as sm
# from mpccontroller import MPCUSV 
import math



class LineofSightfirst():

    def __init__(self,select_path=0):

        self.sideslip=0.0
        #look heading parameters
        self.delta_min=3
        self.delta_max=3
        self.delta_k=1
        self.U_max =1.5
        self.U_min=0.5
        self.y_max=20
        self.chi_max=30
        self.k=0
        self.x_int=0.0
        self.Wpx =[]
        self.Wpy=[]
        self.select_path = select_path
        self.R_calculator =R_Calculator()
        self.pchip_gen = spline_gen()
        # self.obstacle = MPCUSV_colav()
        self.Wp_y_init = []
        self.min_index = 0
        self.x_pose = []
        self.y_pose = []
        self.x_los = 0
        self.y_los =0
        self.x_closest_ =0
        self.y_closest_ =0
        self.curve_slope_angle_next =0
        self.init_x_curve_two  = []
        self.init_y_curve_two  = []
        self.init_x_curve = []
        self.init_y_curve = []
        self.n=0
        self.u =0
        self.v=0

    def path_generate(self,Wpx,Wpy,ds=0.01):
        ds = 0.01
        if self.select_path==0:
            self.coeff ,self.x_init, self.y_init,self.Wp_x_init,self.Wp_y_init= generate_curve(Wpx,Wpy,ds)
            
        elif self.select_path==1:
            pass

    def closest_point(self,eta,x_list,y_list):
        x_v = eta[0]
        y_v = eta[1]

        dx = [x_v-icx for icx in x_list]
        dy = [y_v-icy for icy in y_list]

        distance = [np.sqrt(idx**2 + idy**2) for (idx,idy) in zip(dx,dy)]
        min_value = min(distance)
        self.min_index = distance.index(min_value)

        self.x_closest_ = x_list[self.min_index]
        self.y_closest_ = y_list[self.min_index]

        return self.x_closest_ , self.y_closest_


    def closest_point_minimizer(self,eta,coefficient):
        x_v=eta[0] # current vehicle position x
        y_v=eta[1] # current vehicle position y
        coefficient.sort()

        a,b,c,d=coefficient

        def f(x):
            return a*x**3 + b*x**2 + c*x + d
 
        P = (x_v, y_v)

        def objective(X):
            x, y = X
            return np.sqrt((x - P[0]) ** 2 + (y - P[1]) ** 2)

        def c1(X):
            x, y = X
            return y-f(x)

        c1={'type':'eq','fun':c1}

        X = minimize(objective, x0 = [x_v, y_v],method='SLSQP',constraints= c1)
      
        self.x_closest = X.x[0]  # closest point on curve -x
        self.y_closest = X.x[1]  # closest point on curve -y 

        print('X values:',X)
        print('Cloest X: ',self.x_closest,'Closest y: ',self.y_closest)


    def closest_point_sym(self,eta):

        x_v = eta[0]
        y_v = eta[1]
        coef_ = self.coeff[self.k][:]
        x = sm.Symbol('x')
        deriv = sm.diff(2*(x_v-x)+2*(y_v-coef_[0]*x**3+coef_[1]*x**2+coef_[2]*x+coef_[3]))
        x_candidate = sm.solve(deriv,x)

        return x_candidate


    def execute(self,nu,eta,Wpx,Wpy): 

        x_v = eta[0]
        y_v = eta[1]
        x_velocity = nu[0]
        y_velocity =nu[1]

        m = 5 # Ara?? etraf??ndaki 2 gemi boyundaki ??amber
       
        # dist = np.sqrt((self.x_closest_ -Wpx[self.k+1])**2+(self.y_closest_-Wpy[self.k+1])**2)

        if self.k>=len(Wpx)-2:
            pass
        else:
            dist = np.sqrt((self.x_closest_ -Wpx[self.k+2])**2+(self.y_closest_-Wpy[self.k+2])**2)
        

            if np.sqrt((self.x_closest_ - Wpx[self.k+2])**2+(self.y_closest_-Wpy[self.k+2])**2):
                print('****___------*******')
                if dist<=m:  # Waypoint ge??i??leri i??in
        
                    self.k+=1

                    if self.k>=len(Wpx):
                        pass

        if self.k==len(Wpx):

            self.init_x_curve = self.Wp_x_init[self.k]
            self.init_y_curve = self.Wp_y_init[self.k]
        else:

            self.init_x_curve = list(chain.from_iterable(self.Wp_x_init[self.k:self.k+2]))
            self.init_y_curve = list(chain.from_iterable(self.Wp_y_init[self.k:self.k+2]))

       
        c=self.coeff[self.k].tolist()

        ##############################
        ### Closest Points Methods ### 
        ##############################

        #1# 
        self.closest_point_minimizer(eta,c)
        #2#
        # self.x_closest, self.y_closest = self.closest_point(eta,self.init_x_curve,self.init_y_curve)
        #3#
        #self.closest_point_sym(eta)

        if self.init_y_curve[self.min_index]==self.init_y_curve[-1]:
            curve_slope_angle=self.curve_slope_angle_next
        else:
            curve_slope_angle=math.atan2(self.init_y_curve[self.min_index+1]-self.init_y_curve[self.min_index],self.init_x_curve[self.min_index+1]-self.init_x_curve[self.min_index]) 
        
        self.curve_slope_angle_next=curve_slope_angle
        #cross-track error
        y_e=-(x_v-self.x_closest )*math.sin(curve_slope_angle)+(y_v- self.y_closest)*math.cos(curve_slope_angle) 
  
        # self.var_lookhead_distance=(self.delta_max-self.delta_min)*math.exp(-self.delta_k*y_e**2)+self.delta_min
        path_curv_radius=self.R_calculator.R_cal(eta)
        self.var_lookhead_distance = (abs(y_e)*self.delta_k)+self.delta_min#-path_curv_radius/100

      ####################################################################

        wk = 0
        curve_len =0
        exact_index_location=0
        index_len = 0
       

        if self.k == 0:
            pass
     
        else:
            for m in range(self.k):
                index_len += len(self.Wp_x_init[m])
                          
            exact_index_location =exact_index_location+index_len
                         
        while  curve_len<=self.var_lookhead_distance:

            if wk == 0:
                if self.x_los == self.x_init[-1] and self.y_los == self.y_init[-1]:
                    break
                curve_len+=np.sqrt((self.init_x_curve[self.min_index]-self.init_x_curve[self.min_index+1])**2+(self.init_y_curve[self.min_index]-self.init_y_curve[self.min_index+1])**2)

            if self.k==0:
            
                self.x_los = self.x_init[self.min_index+wk]
                self.y_los = self.x_init[self.min_index+wk]


               
            if self.x_los != self.x_init[-1] and self.y_los != self.y_init[-1]:

                if self.x_init[exact_index_location+self.min_index+wk] ==self.x_init[-1] and self.x_init[exact_index_location+self.min_index+wk==self.y_init[-1]]:
                    break

                curve_len+=np.sqrt((self.x_init[exact_index_location+self.min_index+wk]-self.x_init[exact_index_location+self.min_index+wk+1])**2+(self.y_init[exact_index_location+self.min_index+wk]-self.y_init[exact_index_location+self.min_index+wk+1])**2)
            
                wk+=1
                self.x_los = self.x_init[exact_index_location+self.min_index+wk]
                self.y_los = self.y_init[exact_index_location+self.min_index+wk]

            else:
       
                print('X INIT[[-1]',self.x_init[-1],'Y INIT[[-1]',self.y_init[-1])
                self.x_los = self.x_init[-1]
                self.y_los = self.y_init[-1]
                print('BREAK')
                break

                


        # ########################## NORMAL LOS POINT  ######################################## #
        # self.x_los =  self.x_closest+ self.var_lookhead_distance*math.cos(curve_slope_angle)  #
        # self.y_los =  self.y_closest + self.var_lookhead_distance*math.sin(curve_slope_angle) #
        # ##################################################################################### #
        d = np.sqrt((self.x_closest -self.x_los)**2+(self.y_closest-self.y_los)**2)

        print('actual lookhead',d)
        self.chi_r=math.atan(-y_e/abs(d)) # los angle 

        Beta = math.atan2(y_velocity,x_velocity)
        self.chi_d=curve_slope_angle+self.chi_r-Beta # ref a????
        print('CURVE SLOPE ANGLE',math.degrees(curve_slope_angle))
        print('CH?? R',math.degrees(self.chi_r))

        self.chi_d= math.atan2((self.y_los-y_v),(self.x_los-x_v))
        error_angle=self.chi_d-eta[5] # a???? hatas??

        # referans h??z de??eri
        term1=abs(y_e)/self.y_max
        print('TERM!',term1)
        # print('term1',term1)
        term2=abs(error_angle)/self.chi_max
        print('TERM2',term2)
        U_desired = max(self.U_max*(1-term1-term2),self.U_min)
        
        
        

        # U_desired = speed_generate(path_curv_radius,sapma) 
        print('U_Speed',U_desired)
        print('desired chid',math.degrees(self.chi_d)%360)
        output = dict(error_angle=error_angle,U_desired=U_desired,y_e=y_e,chi_d=self.chi_d,x_los=self.x_los,y_los=self.y_los,Wpy=Wpy[self.k+1],Wpx=Wpx[self.k+1])
        return output

    # def los_simulation(self,eta,Wpx,Wpy,u_control,x_obs,y_obs,current_eta,OS_Arr,TS_Arr):
    def los_simulation(self,eta,Wpx,Wpy,u_control):
        self.x_pose.append(eta[0])
        self.y_pose.append(eta[1])



        plt.clf()
        # plt.cla()
        plt.xlim([0,40])
        plt.ylim([0,40])
        plt.plot(Wpx,Wpy,'ro',label='Waypoints')

        # plt.gcf().gca().add_artist(plt.Circle((x_obs,y_obs),2,fill=False))

        plt.plot(self.x_init,self.y_init,'m--')

        plt.plot(self.x_pose, self.y_pose,"r--")

        # plt.title([u_control[1],u_control[0],self.var_lookhead_distance,current_eta])
        
        plt.plot(eta[0],eta[1],'bo',label='vahicle position')
        plt.title(u_control)
        # plt.plot(OS_Arr[0],OS_Arr[1],'b-.')

        # plt.plot(TS_Arr[0],TS_Arr[1],'c-.')

        plt.plot(self.x_closest,self.y_closest,'co')
        
        plt.plot([eta[0],self.x_closest],[eta[1],self.y_closest],'g--',label='cross track error ')

        plt.plot([self.x_closest,self.x_los],[self.y_closest,self.y_los],'m--',label='Varing Delta')

        plt.plot(self.x_los,self.y_los,'ko')

        plt.plot([eta[0],eta[0]+5*math.cos(eta[5])],[eta[1],eta[1]+5*math.sin(eta[5])])
        

        plt.show(block=False)

        plt.pause(0.0001)

    def animate(self,current_eta,Wpx,Wpy,u_control,i):
        
        anim=FuncAnimation(plt.clf(),self.los_simulation(current_eta,Wpx,Wpy,u_control),interval=1000,blit=True)
    
        plt.show()

        return anim


# if __name__=='__main__':


#     curve = CubicCurve()
#     Wpx=[4,12,20,15,40,65,10]
#     Wpy=[4,20,25,67,45,78,95]



### NOT LOOKHEAD DISTANCE LOS pozisyonu i??in eski yonte myani dikine ile dp??ru ??ekme olay??na vak 