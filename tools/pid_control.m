function [out,cumm] = pid_control(variable,variable_speed,setpoint,kp,ki,kd,delta_time,acum,m)
    error = setpoint - variable;
    ierror = acum + error*delta_time + kd*variable_speed;
    out = kp*error + ki*ierror; 
    cumm = ierror;
end