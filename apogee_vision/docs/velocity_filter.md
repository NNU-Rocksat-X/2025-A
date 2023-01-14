# Velocity Filter

The velocity filter is largely based on the explanation provided by https://www.kalmanfilter.net

# Equations

#### Precursor
- Any variable with a hat ' $\hat{}$ ' means estimate.
- A variable with the subscript n: $x_n$ means the current iteration of the kalman filter loop
- A variable with the subscript n-1: $x_{n-1}$ means the previous iteration of the kalman filter loop
- A variable with the subscript n+1: $x_{n+1}$ means the next iteration of the kalman filter loop
- A Matrix is capitalized and bold face: $\boldsymbol{X}$ while a variable is lower case: $x$
- A Matrix with a superscript T: $\boldsymbol{F}^T$ means [transpose](https://en.wikipedia.org/wiki/Transpose).

#### Variable descriptions
- $\boldsymbol{\hat{x}_n}$ - State vector estimation
- $\hat{x}_n$ - Estimate of the position along the $x$ axis
- $\hat{\dot{x}}_n$ - Estimate of the velocity along the $x$ axis
- $\hat{\ddot{x}}_n$ - Estimate of the acceleration alonx the $x$ axis
- $\boldsymbol{F}$ - State transition matrix
- $dt$ - Change in time
- $\boldsymbol{Q}$ - Process noise uncertainty matrix
- $\sigma_a^2$ - Acceleration variance
- $\boldsymbol{P}_n$ - Uncertainty (covariance) matrix for the current state estimation
- $p_x$ - Variance of the $X$ coordinate position estimate
- $\boldsymbol{R}$ - Measurement uncertainty or Measurement covariance matrix


## Matrix Definitions

### State Vector Estimate
$$
\boldsymbol{\hat{X}}_n = \left[\begin{matrix}
\hat{x}_n\\
\hat{\dot{x}_n}\\
\hat{\ddot{x}}_n\\
\hat{y}_n\\
\hat{\dot{y}}_n\\
\hat{\ddot{y}}_n\\
\hat{z}_n\\
\hat{\dot{z}}_n\\
\hat{\ddot{z}}_n
\end{matrix}\right]
$$

### State Transition Matrix
$$
\boldsymbol{F} = \left[\begin{matrix}
1 & dt & dt^2\over2 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 1 & dt & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & dt & dt^2\over2 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & dt & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & dt & dt^2\over2 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & dt\\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{matrix}\right]
$$

### Process Noise Uncertainty Matrix
$$
\boldsymbol{Q} = \left[\begin{matrix}
dt^4\over4 & dt^3\over2 & dt^2\over2 & 0 & 0 & 0 & 0 & 0 & 0\\
dt^3\over2 & dt^2 & dt               & 0 & 0 & 0 & 0 & 0 & 0\\
dt^2\over2 & dt & 1                  & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & dt^4\over4 & dt^3\over2 & dt^2\over2 & 0 & 0 & 0\\
0 & 0 & 0 & dt^3\over2 & dt^2 & dt               & 0 & 0 & 0\\
0 & 0 & 0 & dt^2\over2 & dt & 1                  & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & dt^4\over4 & dt^3\over2 & dt^2\over2\\ 
0 & 0 & 0 & 0 & 0 & 0 & dt^3\over2 & dt^2 & dt\\              
0 & 0 & 0 & 0 & 0 & 0 & dt^2\over2 & dt & 1                               
\end{matrix}\right] \sigma_a^2
$$

### Estimation Uncertainty Matrix
$$
\boldsymbol{P} = 
    \left[ \begin{matrix}								
        p_{x} & p_{x\dot{x}} & p_{x\ddot{x}} & 0 & 0  & 0 & 0 & 0 & 0\\
        p_{\dot{x}x} & p_{\dot{x}} & p_{\dot{x}\ddot{x}} & 0 & 0 & 0 & 0 & 0 & 0\\
        p_{\ddot{x}x} & p_{\ddot{x}\dot{x}} & p_{\ddot{x}} & 0 & 0 & 0 & 0 & 0 & 0\\
        0 & 0 & 0 & p_{y} & p_{y\dot{y}} & p_{y\ddot{y}} & 0 & 0 & 0 \\
        0 & 0 & 0 & p_{\dot{y}y} & p_{\dot{y}} & p_{\dot{y}\ddot{y}} & 0 & 0 & 0 \\
        0 & 0 & 0& p_{\ddot{y}y} & p_{\ddot{y}\dot{y}} & p_{\ddot{y}} & 0 & 0 & 0 \\
        0 & 0 & 0 & 0 & 0 & 0 & p_{z} & p_{z\dot{z}} & p_{z\ddot{z}}\\
        0 & 0 & 0 & 0 & 0 & 0& p_{\dot{z}z} & p_{\dot{z}}& p_{\dot{z}\ddot{z}}	\\
        0 & 0 & 0 & 0 & 0 & 0& p_{\ddot{z}z}& p_{\ddot{z}\dot{z}}& p_{\ddot{z}}	\\
    \end{matrix}
        \right]
$$

### Measurement Vector

$$
\boldsymbol{z}_n = \left[ \begin{matrix}
x_{measured}\\
y_{measured}\\
z_{measured}
\end{matrix}\right]
$$

### Observation Matrix
Correlates measurement to system state

Measurement $\boldsymbol{z}_n$ has a dimension of $3 \times 1$ and $\boldsymbol{x}_n$ is $9 \times 1$ so $\boldsymbol{H}$ shall be $3 \times 9$.

$$
\boldsymbol{H} = \left[ \begin{matrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\
\end{matrix}\right]
$$

### Measurement Uncertainty Matrix

$$
\boldsymbol{R} = \left[ \begin{matrix}
\sigma_{x_m}^2 & 0 & 0\\
0 & \sigma_{y_m}^2 & 0\\
0 & 0 & \sigma_{z_m}^2
\end{matrix}\right]
$$

### Kalman Gain Matrix

$$
\boldsymbol{K}_n = \left[ \begin{matrix}
K_{0,1_n} & K_{0,2_n} & K_{0,3_n}\\ 
K_{1,1_n} & K_{1,2_n} & K_{1,3_n}\\  
K_{2,1_n} & K_{2,2_n} & K_{2,3_n}\\  
K_{3,1_n} & K_{3,2_n} & K_{3,3_n}\\  
K_{4,1_n} & K_{4,2_n} & K_{4,3_n}\\  
K_{5,1_n} & K_{5,2_n} & K_{5,3_n}\\  
K_{6,1_n} & K_{6,2_n} & K_{6,3_n}\\  
K_{7,1_n} & K_{7,2_n} & K_{7,3_n}\\  
K_{8,1_n} & K_{8,2_n} & K_{8,3_n}\\  
\end{matrix} \right]
$$

## Equations

### Predict Step Equations

<p align="center">
State extrapolation
</p>


$$
\boldsymbol{\hat{X}}_{n+1} = \boldsymbol{F}\boldsymbol{\hat{X}}_{n}
$$


<p align="center">
Covariance Extrapolation equation	 
</p>

$$
\boldsymbol{P}_{n+1} = \boldsymbol{F}\boldsymbol{P}_n\boldsymbol{F}^T + \boldsymbol{Q}
$$

### Measurement Update Step Equations

<p align="center">
Measurement Update Equation
</p>

$$
z_n = \boldsymbol{H}x_n
$$

$$
\left[ \begin{matrix}
x_{measured}\\
y_{measured}\\
z_{measured}
\end{matrix} \right] =
\left[ \begin{matrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\
\end{matrix}\right]
\left[\begin{matrix}
\hat{x}_n\\
\hat{\dot{x}_n}\\
\hat{\ddot{x}}_n\\
\hat{y}_n\\
\hat{\dot{y}}_n\\
\hat{\ddot{y}}_n\\
\hat{z}_n\\
\hat{\dot{z}}_n\\
\hat{\ddot{z}}_n
\end{matrix}\right]
$$

<p align="center">
Kalman Gain
</p>

$$
\boldsymbol{K}_{n} = \boldsymbol{P}_{n,n-1}\boldsymbol{H}^{T}\left(\boldsymbol{HP}_{n,n-1}\boldsymbol{H}^{T} + \boldsymbol{R} \right)^{-1}
$$


$$
\left[ \begin{matrix}
K_{0,1_n} & K_{0,2_n} & K_{0,3_n}\\ 
K_{1,1_n} & K_{1,2_n} & K_{1,3_n}\\  
K_{2,1_n} & K_{2,2_n} & K_{2,3_n}\\  
K_{3,1_n} & K_{3,2_n} & K_{3,3_n}\\  
K_{4,1_n} & K_{4,2_n} & K_{4,3_n}\\  
K_{5,1_n} & K_{5,2_n} & K_{5,3_n}\\  
K_{6,1_n} & K_{6,2_n} & K_{6,3_n}\\  
K_{7,1_n} & K_{7,2_n} & K_{7,3_n}\\  
K_{8,1_n} & K_{8,2_n} & K_{8,3_n}\\  
\end{matrix} \right] = 
$$

$$
\left[ \begin{matrix}
p_{x_{n-1}} & p_{x\dot{x}_{n-1}} & p_{x\ddot{x}_{n-1}} & 0 & 0  & 0 & 0 & 0 & 0\\
p_{\dot{x}x_{n-1}} & p_{\dot{x}_{n-1}} & p_{\dot{x}\ddot{x}_{n-1}} & 0 & 0 & 0 & 0 & 0 & 0\\
p_{\ddot{x}x_{n-1}} & p_{\ddot{x}\dot{x}_{n-1}} & p_{\ddot{x}_{n-1}} & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & p_{y_{n-1}} & p_{y\dot{y}_{n-1}} & p_{y\ddot{y}_{n-1}} & 0 & 0 & 0 \\
0 & 0 & 0 & p_{\dot{y}y_{n-1}} & p_{\dot{y}_{n-1}} & p_{\dot{y}\ddot{y}_{n-1}} & 0 & 0 & 0\\
0 & 0 & 0 & p_{\ddot{y}y_{n-1}} & p_{\ddot{y}\dot{y}_{n-1}} & p_{\ddot{y}_{n-1}} & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & p_{z_{n-1}} & p_{z\dot{z}_{n-1}} & p_{z\ddot{z}_{n-1}}\\
0 & 0 & 0 & 0 & 0 & 0 & p_{\dot{z}z_{n-1}} & p_{\dot{z}_{n-1}} & p_{\dot{z}\ddot{z}_{n-1}}\\
0 & 0 & 0 & 0 & 0 & 0& p_{\ddot{z}z_{n-1}}& p_{\ddot{z}\dot{z}_{n-1}}& p_{\ddot{z}_{n-1}}
\end{matrix}\right] \times
$$

$$
\left[ \begin{matrix}
1 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 1\\
0 & 0 & 0\\
0 & 0 & 0\\
\end{matrix}\right] \times
\left(\left[ \begin{matrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\
\end{matrix}\right]
\left[ \begin{matrix}								
p_{x_{n-1}} & p_{x\dot{x}_{n-1}} & p_{x\ddot{x}_{n-1}} & 0 & 0  & 0 & 0 & 0 & 0\\
p_{\dot{x}x_{n-1}} & p_{\dot{x}_{n-1}} & p_{\dot{x}\ddot{x}_{n-1}} & 0 & 0 & 0 & 0 & 0 & 0\\
p_{\ddot{x}x_{n-1}} & p_{\ddot{x}\dot{x}_{n-1}} & p_{\ddot{x}_{n-1}} & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & p_{y_{n-1}} & p_{y\dot{y}_{n-1}} & p_{y\ddot{y}_{n-1}} & 0 & 0 & 0\\
0 & 0 & 0 & p_{\dot{y}y_{n-1}} & p_{\dot{y}_{n-1}} & p_{\dot{y}\ddot{y}_{n-1}} & 0 & 0 & 0\\
0 & 0 & 0& p_{\ddot{y}y_{n-1}} & p_{\ddot{y}\dot{y}_{n-1}} & p_{\ddot{y}_{n-1}} & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & p_{z_{n-1}} & p_{z\dot{z}_{n-1}} & p_{z\ddot{z}_{n-1}}\\
0 & 0 & 0 & 0 & 0 & 0& p_{\dot{z}z_{n-1}} & p_{\dot{z}_{n-1}}& p_{\dot{z}\ddot{z}_{n-1}}\\
0 & 0 & 0 & 0 & 0 & 0& p_{\ddot{z}z_{n-1}}& p_{\ddot{z}\dot{z}_{n-1}}& p_{\ddot{z}_{n-1}}\\
\end{matrix}\right]
\left[ \begin{matrix}
1 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 0\\
0 & 0 & 0\\
0 & 0 & 1\\
0 & 0 & 0\\
0 & 0 & 0\\
\end{matrix}\right] + 
\left[ \begin{matrix}
\sigma_{x_m}^2 & 0 & 0\\
0 & \sigma_{y_m}^2 & 0\\
0 & 0 & \sigma_{z_m}^2
\end{matrix}\right]\right)^{-1}
$$

<p align="center">
Update State Estimate
</p>

$$
\boldsymbol{\hat{x}}_{n,n} = \boldsymbol{\hat{x}}_{n,n-1} + \boldsymbol{K}_{n} (\boldsymbol{z}_{n} - \boldsymbol{H\hat{x}}_{n,n-1} )
$$

<p align="center">
Update Estimate Uncertainty (Covariance)
</p>

$$
\boldsymbol{P}_{n,n} = \left(\boldsymbol{I} - \boldsymbol{K}_{n}\boldsymbol{H} \right) \boldsymbol{P}_{n,n-1} \left(\boldsymbol{I} - \boldsymbol{K}_{n}\boldsymbol{H} \right)^{T} + \boldsymbol{K}_{n}\boldsymbol{R}\boldsymbol{K}_{n}^{T}
$$
