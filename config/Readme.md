# INS Configuration
insmode=TCINS \
outputpath=./OUTDATA/TCLOG\
pitch0=-0.1241 for initial state (deg)\
roll0=-0.0064 for initial state (deg)\
yaw0=0.9482 for initial state (deg)\
vn0x=0.05 for initial state (m / s)\
vn0y=0.05 for initial state (m / s)\
vn0z=0.05 for initial state (m / s)\
latitude0=0.559538705118410 for initial state (rad)\
longitude0=2.073217070241812 for initial state (rad)\
height0=13.0 for initial state (m)\
gyrobias0x=0.00 for initial state (rad / s)\
gyrobias0y=0.00 for initial state (rad / s)\
gyrobias0z=0.00 for initial state (rad / s)\
accbias0x=0.00 for initial state (m / s^2)\
accbias0y=0.00 for initial state (m / s^2)\
accbias0z=0.00 for initial state (m / s^2)\
lever0x=0.01 for initial state (m)\
lever0y=0.01 for initial state (m)\
lever0z=0.01 for initial state (m)\
dt=0.005 for initial state (s)\
clockcorrection0=10000 for initial state (m)\
clockdrift0=0 for initial state (m/s)\
imufs=250 IMU sample frequency (Hz)\
devGyroV=4.3633e-05 for Q matrix (rad/sqrt(Hz))\
devAccV=0.01333 for velocity randow walk Q matrix m/s^2/sqrt(Hz) 0.8m/s/sqrt(h) - stim300 80g\
devAccU=0.004 for bias instability Q matrix m/s^2 0.4mg - stim300 80g can not be ignored! \
clock_phase_PSD=1.0 for Q matrix Code tracking error SD (m) \
clock_freq_PSD=0.2 for Q matrix  Range rate tracking error SD (m/s) \
rx_clock_offset=10000 for P matrix Receiver clock offset at time=0 (m)\
rx_clock_drift=100 for P matrix  Receiver clock drift at time=0 (m/s)\
pseudo_range_SD=5.5 for R matrix Pseudo-range measurement noise SD (m)\
range_rate_SD=0.01 for R matrix Pseudo-range rate measurement noise SD (m/s)\
initialBiasG=250 for P matrix (deg / h)\
initialBiasA=0.0073 for P matrix (m / s^2)\
NHCR=0.5 for car vehicle NHC R