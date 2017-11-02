# Flow_Control_Slave
The arduino nano code for liquid rate control

Autosteer_Flow_Master contains the AgOpenGps arduino code with a couple minor modifications to send speeed and relay data to a slave arduino nano.
The slave computes a liquid application rate by reading a flowmeter, adjusts a valve to hit the target rate.
Slave then sends back the actual current rate in liters/hec to the master.

Slave file utilizes a generic pulse reading class written by Torriem to help compute the liquid flow rate.
