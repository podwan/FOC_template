#include "pid.h"
#include "time_utils.h"
#include "math_utils.h"
#include "comm.h"
#include "bldcMotor.h"


void pidInit(PidController *PID, float kp, float ki, float kd, float output_ramp, float outMin, float outMax)
{
   PID->P = kp;
   PID->I = ki;
   PID->D = kd;
   PID->integral_prev = 0;
   PID->error_prev = 0;
   PID->output_prev = 0;
   PID->output_ramp = output_ramp;
   PID->outMin = outMin;
   PID->outMax = outMax;
   PID->timestamp_prev = micros();
}

float PID_operator(PidController *PID, float error)
{
   unsigned long timestamp_now;
   float Ts;
   float proportional, integral, derivative, output;
   float output_rate;
   timestamp_now = micros();
   Ts = (timestamp_now - PID->timestamp_prev) * 1e-6f;

   if (Ts <= 0 || Ts > 0.5f)
      Ts = 1e-3f;

   // proportional part
   proportional = PID->P * error; // P
   // Tustin transform of the integral part
   integral = PID->integral_prev + PID->I * Ts * 0.5f * (error + PID->error_prev);
   // antiwindup - limit the output
   integral = CONSTRAINT(integral, PID->outMin, PID->outMax);
   // Discrete derivation
   derivative = PID->D * (error - PID->error_prev) / Ts;
   // sum all the components
   output = proportional + integral + derivative;
   output = CONSTRAINT(output, PID->outMin, PID->outMax);
   // if output ramp defined
   if (PID->output_ramp > 0)
   {
      // Limit the acceleration by ramping the output
      output_rate = (output - PID->output_prev) / Ts;
      if (output_rate > PID->output_ramp)
         output = PID->output_prev + PID->output_ramp * Ts;
      else if (output_rate < -PID->output_ramp)
         output = PID->output_prev - PID->output_ramp * Ts;
   }

#if CALI_PID
   if (PID == &bldcMotor.pidVelocity)
   {
      comm1 = timestamp_now;
      comm2 = PID->integral_prev;
      comm3 = PID->timestamp_prev;
      comm4 = Ts;
      comm5 = derivative;
      comm6 = PID->error_prev;
      comm7 = output;
      comm8 = shaftVelocity;
      comm9 = error;
      comm10 = PID->outMin;
      comm11 = PID->outMax;
   }
#endif
   // saving for the next pass
   PID->integral_prev = integral;
   PID->output_prev = output;
   PID->error_prev = error;
   PID->timestamp_prev = timestamp_now;

   return output;
}
