# AOG_Rate
The arduino nano code for liquid rate control

AOG_Rate contains the AgOpenGps arduino code which computes "liters per minute" by reading a flowmeter, adjusts a valve to hit the target rate, then sends back the actual current rate in liters/min to the master.
Also reads a pressure transducer and calculates a bar pressure of the system.

File utilizes a generic pulse reading class written by Torriem to help compute the liters per minute.
