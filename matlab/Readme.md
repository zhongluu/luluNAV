# Simulation with Matlab
## Dependency
1. Running in Matlab 2022b
2. Tightly coupled navigation depend on the psins toolbox that can be achieved by https://www.psins.org.cn/ or https://github.com/WangShanpeng/PSINS

## File Description
	|-- ..matlab
	|-- .tightlyCoupleNavigation
		|-- EX_of_MineBIVBTCNavEKF21.m // sequential tightly coupled with the novel fitler
		|-- EX_of_MineTCChiNavEKF21.m // sequential tightly coupled with Chi-square method
		|-- EX_of_MineTCNavEKF21.m // sequential tightly coupled with normal EKF
	|-- .PrincipleVerifyByROTModel
		|-- test_ln_VBEKFSEQ_DMN_4d.m // linear meausurement model with the novel filter by sequential processing
		|-- test_radar_VBSEQ_CHI_DMN_5d.m // nonlinear measurement model with the novel filter by sequential processing
	|-- ResComp
		|-- COMP21.m // a compare script for various tightly coupled navigation method estimatin result
		|-- COMPTIME // a compare script for various tightly coupled navigation method running time
		|-- *.mat // the filter result of various methods 
	|-- .img // PrincipleVerify result images
	|-- *.mat // the raw data of the tightly coupled navigation

## PrincipleVerify Result
### For linear model with sequential processing VB estimation
State:

![LMVBS](./img/LMVBTRJ.jpg "State")

R Estimation:

![LMVBS](./img/LMVBREstimation.jpg "R")

CRLB:

![LMVBS](./img/LMVBCRLB.jpg "CRLB")

Iteration:

![LMVBS](./img/LMVBIteration.jpg "Iter")

RMSE State:

![LMVBS](./img/LMVBStateRMSE.jpg "RMSE State")

RMSE R:

![LMVBS](./img/LMVBMeasuremtRMSE.jpg "RMSE R")

### For nonlinear model with sequential processing VB estimation base Bernoulli and IG distribution

State:

![NMVBS](./img/NMVBTRJ.jpg "NState")

R Estimation:

![NMVBS](./img/NMVBREstimation.jpg "NR")

Bernoulli Estimation:

![NMVBS](./img/NMVBBern.jpg "NBernoulli")

CRLB:

![NMVBS](./img/NMVBCRLB.jpg "NCRLB")

Iteration:

![NMVBS](./img/NMVBIteration.jpg "NIter")

RMSE State:

![NMVBS](./img/NMVBStateRMSE.jpg "RMSE NState")

RMSE R:

![NMVBS](./img/NMVBMeasuremtRMSE.jpg "RMSE NR")

## TightlyCoupleNavigation Result Comparison
Attitude:

![VBAttitude](./img/AttiComp.jpg "Attitude")

Velocity:

![VBVelocity](./img/VelComp.jpg "Velocity")

Position:

![VBPosition](./img/PosComp.jpg "Position")