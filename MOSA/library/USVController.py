#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy import clip
from interpolation_ import OtterPolation
import math

class USVController():
    def __init__(self):
        self.prev_error = 0
        self.error = 0
        self.prev_heading_error = 0
        self.prev_speed_error = 0
        self.delta_error = 0
        self.saturation_limit = 0
        self.U_i_heading = 0
        self.U_i_speed = 0
        self.U_i_old = 0
        self.filtred_signal = 0.1
        self.filtred_heading_signal = 0.01
        self.pre_speed_ref = 0
        self.saturation_limit=104
        self.U_i =0
        self.clamp=False
        self.heading_clamp=False
        self.U_i_prev = 0
        self.intplt = OtterPolation() 
        self.heading_error=0
        self.prev_U = 0
    def Heading_controller(self,ref_heading,current_eta,Kp=0,Ki=0,Kd=0):

        
        feed_back = math.degrees(current_eta[-1])
        self.heading_error = ref_heading-feed_back

        while self.heading_error<-180:
            self.heading_error = self.heading_error +360
        while self.heading_error > 180:
            self.heading_error = self.heading_error -360

        # Proportion Term
        U_p = Kp*self.heading_error

        # Integral Term
        if self.heading_clamp==False:
            self.U_i_heading=self.U_i_heading+Ki*self.heading_error
        elif self.heading_clamp==True:
            pass
            # Control Signal 
        

        # Derivative Term
        U_d = Kd*(self.heading_error-self.prev_heading_error)

        
        Control_signal = U_p + U_d + self.U_i_heading
        
    
        self.prev_heading_error  = self.heading_error

        return Control_signal


        
    def Speed_controller(self,ref_speed,velocity,Kp=0,Ki=0,Kd=0,Kf=0,saturation_method=0,pid_method=0):

        self.speed_error = ref_speed-velocity
        delta_error = self.speed_error-self.prev_error

        # Feedforward Term
        U_f = self.intplt.interpolation(ref_speed)
        if pid_method == 0:
            # Velocity PID
            # Propotion Term
            self.vU_p=Kp*delta_error
            # Integral Term
            self.vU_i = Ki*self.speed_error
            # Control Signal 

            self.U = U_f+self.vU_p+self.vU_i#+self.prev_U
            self.prev_U = self.U
            return self.U
        if pid_method==1:
            # Positional PID
            # Proportion Term 
            self.U_p = Kp*self.speed_error
            # Derivative Term 
            self.U_d = Kd*(self.speed_error-self.prev_speed_error)
            self.prev_speed_error = self.speed_error
            # Integral Term
            if saturation_method  == 0:
                if self.clamp==False:
                    self.U_i=self.U_i+Ki*self.speed_error
                elif self.clamp==True:
                    pass
                # Control Signal 
                self.U = self.U_p + self.U_d + self.U_i+U_f
                return self.U 
            if saturation_method ==1 :
                self.U_i=self.U_i_prev+Ki*self.speed_error
                self.U_i_prev=self.U_i
                self.U = self.U_p + self.U_d + self.U_i+U_f
                # self.U_sat = max(min(-self.saturation_limit),self.saturation_limit)
                self.U_sat=sorted((-self.saturation_limit, self.U, self.saturation_limit))[1]
                if self.U!=self.U_sat:
                    self.U_i_prev = self.U_sat-self.U_p-U_f


                
                self.prev_error = self.speed_error
                return self.U_sat

        


    def control_allocation(self,u_avr,u_diff):
        max_frw_rpm = 104
        max_rvs_rpm = -104
        global u_control

        if u_avr>max_frw_rpm:
            u_avr=max_frw_rpm
        elif u_avr<max_rvs_rpm:
            u_avr=max_rvs_rpm
        # print('u_avr---',u_avr)
        n1=u_avr-u_diff
        n2=u_avr+u_diff

        if n1>max_frw_rpm:
            n1=max_frw_rpm
        if n1<max_rvs_rpm:
            n1=max_rvs_rpm
        if n2>max_frw_rpm:
            n2=max_frw_rpm   
        if n2<max_rvs_rpm:
            n2=max_rvs_rpm

        u_control=[n1,n2]
        
        return u_control
    
    def reset_integral(self,U):

        if U>self.saturation_limit:
            U=self.saturation_limit
            self.clamp=True
        elif U<-self.saturation_limit:
            U=-self.saturation_limit
            self.clamp=True
        else:
            self.clamp=False
       
        return U

    def reset_integral_heading(self,U):

        if U>self.saturation_limit:
            U=self.saturation_limit
            self.heading_clamp=True
        elif U<-self.saturation_limit:
            U=-self.saturation_limit
            self.heading_clamp=True
        else:
            self.heading_clamp=False
       
        return U


    def Filtred_speed_signal(self,speed_ref):

        delta_rate = 0.03
        if self.pre_speed_ref == self.filtred_signal:
            self.pre_speed_ref = self.filtred_signal
        else:
            if self.filtred_signal<speed_ref:
                self.filtred_signal += +delta_rate
                if self.filtred_signal>speed_ref:
                    self.filtred_signal=speed_ref

            elif self.filtred_signal>speed_ref:
                self.filtred_signal += - delta_rate 
                if self.filtred_signal < speed_ref:
                    self.filtred_signal = speed_ref
            else:
                self.filtred_signal=speed_ref
        return self.filtred_signal


    def Filtred_heading_referans(self,heading_ref):

        if heading_ref-self.filtred_heading_signal<0:
            if (heading_ref-self.filtred_heading_signal)%360<(self.filtred_heading_signal-heading_ref):
                self.filtred_heading_signal = self.filtred_heading_signal+0.36

                if self.filtred_heading_signal>0:

                    if (-self.filtred_heading_signal)%360>heading_ref:
                        self.filtred_heading_signal=heading_ref
            elif  (heading_ref-self.filtred_heading_signal)%360>(self.filtred_heading_signal-heading_ref):

                self.filtred_heading_signal = self.filtred_heading_signal - 0.36
                if self.filtred_heading_signal<0:
                    if (self.filtred_heading_signal%360)>heading_ref:
                        self.filtred_heading_signal=heading_ref
        elif heading_ref-self.filtred_heading_signal>0:
            if (heading_ref-self.filtred_heading_signal)<(self.filtred_heading_signal-heading_ref)%360:
                self.filtred_heading_signal = self.filtred_heading_signal+0.36
                if (self.filtred_heading_signal)>0:
                    if self.filtred_heading_signal>heading_ref:
                        self.filtred_heading_signal=heading_ref
            elif  (heading_ref-self.filtred_heading_signal)>(self.filtred_heading_signal-heading_ref)%360:

                self.filtred_heading_signal = self.filtred_heading_signal - 0.36
                if self.filtred_heading_signal<0:

                    if (self.filtred_heading_signal%360)<heading_ref:
                            self.filtred_heading_signal=heading_ref
                
        return self.filtred_heading_signal


    
            



         
         


        

