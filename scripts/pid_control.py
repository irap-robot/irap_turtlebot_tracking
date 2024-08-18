import numpy as np 

class PID_PositionControl : 
    def __init__(self) :
        self.P = 0 
        self.I = 0 
        self.D = 0 

        self.error = 0 
        self.error_previous = 0 
        self.sum_error = 0
        self.max_sum_error = 1000 
        self.min_error = 0 

        self.max_output = 10000

    def set_gain(self,p,i,d) : 
        self.P = p 
        self.I = i 
        self.D = d 

    def set_max_sum_error(self,max_sum_error) : 
        self.max_sum_error = max_sum_error 

    def set_min_error(self,min_error) : 
        self.min_error = min_error 

    def set_max_output(self,max_output) : 
        self.set_max_output = max_output

    def get_gain(self) : 
        return np.array([self.P,self.I,self.D])
    
    def get_error(self) : 
        return np.array([self.error,self.sum_error,self.error_previous])
    
    def control(self,current,setpoint) : 
        self.error = current - setpoint
        
        if abs(self.error) < self.min_error : 
            self.error = 0 
            self.sum_error = 0 
        
        output = self.error*self.P + self.sum_error*self.I + (self.error - self.error_previous)*self.D

        if abs(output) > self.max_output : 
            if output > 0 : 
                output = self.max_output 
            elif output < 0 : 
                output = -self.max_output 
        
        self.error_previous = self.error
        self.sum_error += self.error 

        if abs(self.sum_error) >= self.max_sum_error : 
            if self.sum_error > 0 : 
                self.sum_error = self.max_sum_error 
            elif self.sum_error < 0 : 
                self.sum_error = -self.max_sum_error 

        return output 
    
    