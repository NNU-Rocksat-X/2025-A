# Angular Velocity Filter

The angular velocity filter is largely based on the explanation provided by https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf

# Equations

#### Precursor
- Any variable with a hat ' $\hat{}$ ' means estimate.
- A variable with the subscript n: $x_n$ means the current iteration of the kalman filter loop
- A variable with the subscript n-1: $x_{n-1}$ means the previous iteration of the kalman filter loop
- A variable with the subscript n+1: $x_{n+1}$ means the next iteration of the kalman filter loop
- A Matrix is capitalized and bold face: $\boldsymbol{X}$ while a variable is lower case: $x$
- A Matrix with a superscript T: $\boldsymbol{F}^T$ means [transpose](https://en.wikipedia.org/wiki/Transpose).

#### Variable descriptions
- $\boldsymbol{\hat{X}_n}$ - State vector estimation
- $\hat{q}_n$ - Estimation of the orientation represented as a unit quaterion.
    - Where $q = q_0 + q_1i + q_2j + q_3k$
    - $q_0$ is a scalar representing the rotation around an axis represented by the vector $\vec{q} = (q_1i + q_2j + q_3k)$
    - $\Phi$ is the angle of this rotation
    - $q_0 = cos({\Phi\over2})$
    - $q_1 = sin({\Phi\over2})i$
    - $q_2 = sin({\Phi\over2})j$
    - $q_3 = sin({\Phi\over2})k$
- $\boldsymbol{\hat{w}}$ - Estimation of the angular velocity vector
- $\hat{w}_{xn}$ - Estimation of the angular velocity around the x axis
- $\hat{w}_{yn}$ - Estimation of the angular velocity around the y axis
- $\hat{w}_{zn}$ - Estimation of the angular velocity around the z axis
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
\hat{q_1}_n\\
\hat{q_2}_n\\
\hat{q_3}_n\\
\hat{q_4}_n\\
\hat{w_x}_n\\
\hat{w_y}_n\\
\hat{w_z}_n
\end{matrix}\right]
$$

### State Transition Matrix

Since $\boldsymbol{\hat{w}}_{n+1} = \boldsymbol{\hat{w}}_{n}$ the angular velocity portion of the state transition matrix is as follow:

$$
\left[\begin{matrix}
\hat{w_x}_{n+1}\\
\hat{w_y}_{n+1}\\
\hat{w_z}_{n+1}
\end{matrix}\right] =
\left[\begin{matrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{matrix}\right]
\left[\begin{matrix}
\hat{w_x}_{n}\\
\hat{w_y}_{n}\\
\hat{w_z}_{n}
\end{matrix}\right]
$$

The change in angle $\alpha_\Delta = |\boldsymbol{\hat{w}}_n| \cdot \Delta t$ 
and the axis $\Delta\vec{e} = {\boldsymbol{\hat{w}}_n\over|\boldsymbol{\hat{w}}_n|}$

$$\Delta q = 
\left[\begin{matrix}
cos({\alpha_\Delta \over2})\\
sin({\alpha_\Delta\over2}) \Delta\vec{e}
\end{matrix}\right]
$$

$$
 \vec{q}_{n+1} = \vec{q}_n \Delta q
$$
Note: This does is not matrix multiplication (Must follow quaternion multiplication in equations 1 to 3 in paper)

The unscented kalman filter implements the process model as some function $A()$. The next state can be predicted with:

$$
x_{k+1} = A(x_k, \mathrm{w}_k) = 
 \left[\begin{matrix}
q_{k} q_{\mathrm{w}}q_{\Delta}\\
\vec{w}_{k_x} + \vec{\mathrm{w}}_{w_x}\\
\vec{w}_{k_y} + \vec{\mathrm{w}}_{w_y}\\
\vec{w}_{k_z} + \vec{\mathrm{w}}_{w_z}\\
\end{matrix}\right]
$$





### Process Noise Uncertainty Matrix


TODO: Some 3x3 matrix representing the covariance of the random variable w representing the process noise
$$
\boldsymbol{Q} = \left[\begin{matrix}
? & ? & ? & ? & ? & ?\\
? & ? & ? & ? & ? & ?\\  
? & ? & ? & ? & ? & ?\\  
? & ? & ? & ? & ? & ?\\  
? & ? & ? & ? & ? & ?\\
? & ? & ? & ? & ? & ?\\    
\end{matrix}\right] \sigma_a^2
$$

Process Noise
$$
W_k = \left[\begin{matrix}
\mathrm{w}_{q_x}\\
\mathrm{w}_{q_y}\\
\mathrm{w}_{q_z}\\
\mathrm{w}_{w_x}\\
\mathrm{w}_{w_y}\\
\mathrm{w}_{w_z}\\
\end{matrix}\right]
$$

$W_k$ must be converted to a unit quaternion in order to be added to the four component quaternion:

The change in angle $\alpha_\mathrm{w} = |\vec{\mathrm{w}}_q| $ 
and the axis $\vec{e}_{\mathrm{w}} = {\boldsymbol{\vec{\mathrm{w}}}_q\over|\vec{\mathrm{w}}_q|}$

The noise quaternion is:

$$q_{\mathrm{w}} = 
\left[\begin{matrix}
cos({\alpha_\mathrm{w} \over2})\\
e_{\mathrm{w}_x}sin({\alpha_\mathrm{w}\over2})\\
e_{\mathrm{w}_y}sin({\alpha_\mathrm{w}\over2})\\
e_{\mathrm{w}_z}sin({\alpha_\mathrm{w}\over2})
\end{matrix}\right]
$$


### Sigma Points
The kinematics of quaternion angles (and any angle) are calculated with non-linear functions so a normal Kalman Filter cannot be used. This algorithm uses an Unscented Kalman Filter which means instead of using the covariance of a gaussian distribution to represent the state and uncertainty, a distribution of points (Sigma Points) are generated to estimate the next state and covariance.

If the uncertainty matrix has the shape $n\times n$ then $2n$ sigma points are generated.

The points must have the same mean and covariance as $P_{n-1}$
#### Point generation
1. A matrix $S$ is computed with the property $P_{k-1} = S^TS$

    $S$ is essentially the square root of $P_{k-1}$

    $S$ is computed with the Cholesky Decompotition

    However, we must take into account the process noise with covariance $Q$ so $Q$ is added to the state covariance $P_n$ before calculating the sigma points.

    $$
        S = cholesky(P_{k-1}+Q)
    $$

2. Multiply the column vectors by +/- sqrt(2n) to get the set $\{\mathcal{W}_i\}$

    $Wi$ is a vector with six dimensions because the estimation uncertainty is 6x6 and the model has six degrees of freedom.

3. Calculate sigma points with $\mathcal{X}_i = \hat{x}_{n-1} + \mathcal{W}_i$
    
    Note that $\hat{x}_{n-1}$ is a vector with 7 dimensions and $\mathcal{W}_i$ is a vector with 6 dimensions. We must perform this calculation with the quaternion form of these vectors:

$$
\mathcal{X}_i  = \left( \begin{matrix}
q_{k-1} q_{\mathcal{W}}\\
\vec{w}_{k-1} + \vec{w}_\mathcal{W}
\end{matrix}\right)
$$

Where $q_{\mathcal{W}}$ is the quaternion corresponding to the first three components of $\mathcal{W}_i$ and $\vec{w}_\mathcal{W}$ is the angular velocity vector built from the last three components of $\mathcal{W}_i$.

The sigma points $\{\mathcal{X}_i\}$ are then projected ahead of time using the process model $A()$ to create a new set of projected points $\mathcal{Y}_i$.

$$
\mathcal{Y}_i = A(\mathcal{X}_i, 0)
$$

The 0 means no additional noise is added.

##### Mean projection Calculation

Next the mean of the projections $\hat{x}^-_k$ must be calculated, but because the orientation vectors are quaternions you cannot simply add and divide (Called the barycentric mean). 

$$
\hat{x}^-_k = mean(\{\mathcal{Y}_i\})
$$

We can used intrinsic gradient decent to calculate the mean.

First some background, you can calculate the rotation $q_{12}$ between two quaternions $q1$ and $q2$ with 
$$
q_{12} =  q_2q_1^{-1}
$$

We can iteratively calculate the estimation of the mean orientation $\bar{q}$ by calculating an error quaternion $e_i$ for each element of the sigma points set.

$$
e_i = q_i\bar{q}_t^-1
$$

where $e_i$ is the error quaternion, $q_i$ is a quaternion in the sigma points set $\mathcal{Y_i}$ and $\bar{q}_t^-1$ is the previous estimation of the mean orientation. $\bar{q}_t^-1$ can be initialized with any arbitraty values, but initializing it with the orientation from the previous state vector estimate will reduce the number of iterations.
### Estimation Uncertainty Matrix
To calculate the estimation uncertainty matrix (aka Priori State Vector Covariance), the difference between each projected sigma point and the mean of the distribution $\mathcal{W}'_i$ must be calculated.

$$
\mathcal{W}'_i = \left( \begin{matrix}
\vec{r}_{\mathcal{W}'}\\
\vec{w}_{\mathcal{W}'}
\end{matrix} \right)
$$

Where $\vec{r}_{\mathcal{W}'}$ is the difference of the angular velocity components of  each projected sigma point $\{\mathcal{Y_i}\}$ and the mean of the projections $\hat{x}^-_k$ and $\vec{w}_{\mathcal{W}'}$ is the difference in rotation which was already calculated.

$$
\vec{w}_{\mathcal{Wi}'} = \vec{e}_i
$$

Then the Priori State Vector Covariance can be calculated with:

$$
\boldsymbol{P}^-_k = {1\over2n} \sum_{i=1}^{2n}\mathcal{W_i}'\mathcal{W_i}'^T
$$

Since $\mathcal{W_i}'$ is a 6 dimensional vector, multiplying by the transpose $\mathcal{W_i}'^T$ creates a $6\times6$ matrix $P^-_k$.

$$
\boldsymbol{P}^-_k = 
    \left[ \begin{matrix}								
p & p & p & p & p & p\\
p & p & p & p & p & p\\
p & p & p & p & p & p\\
p & p & p & p & p & p\\
p & p & p & p & p & p\\
p & p & p & p & p & p\\
    \end{matrix}
        \right]
$$





### Measurement Vector

$$
\boldsymbol{z}_n = \left[ \begin{matrix}
q_{1_{measured}}\\
q_{2_{measured}}\\
q_{3_{measured}}\\
q_{4_{measured}}
\end{matrix}\right]
$$

### Observation Matrix
Correlates measurement to system state.

Note: This is different than the quaternion UKF orientation tracking paper.

Measurement $\boldsymbol{z}_n$ has a dimension of $4 \times 1$ and $\boldsymbol{x}_n$ is $6 \times 1$ so $\boldsymbol{H}$ shall be $4 \times 7$.

$$
\boldsymbol{H} = \left[ \begin{matrix}
1 & 0 & 0 & 0 & 0 & 0 & 0\\
0 & 1 & 0 & 0 & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0 & 0\\
\end{matrix}\right]
$$

During the update step, the estimate $z_k^-$ of the measurement must also be calculated.

$$
z_k^- = mean(\{\mathcal{Z}_i\})
$$

$$
\mathcal{Z}_i = H(\mathcal{Y}_i, 0)
$$

Where $\mathcal{Z}_i$ is a distribution of predicted measurements, $H$ is a function that relates the measurement value $z$ to the state vector $x$ given a random variable $v$ measurement noise. For this step, since there is no measurement noise $H(\mathcal{Y}_i, 0) = \boldsymbol{H}\mathcal{Y}_i$

The reasoning behind this is the sigma points $\{\mathcal{X}_i\}$ (distribution of the current estimated state) were projected into the future to get $\{\mathcal{Y}_i\}$ (distribution of the predicted next state). The state contains a quaternion representing the orientation of the object, but the orientation is directly measured (although it is measured noisely). Therefore, the distribution of the estimate $\{\mathcal{Z}_i\}$ is equal to the quaternion orientation in the predicted state distribution.

The innovation $v_k$ can then be calculated with:

$$
v_k = z_k - z_k^-
$$

Where $z_k$ is the actual measurement (the measured quaternion orientation of the object).

### Measurement Uncertainty Matrix

$$
\boldsymbol{R} = \left[ \begin{matrix}
\sigma_{q_m}^2 & 0 & 0 & 0\\
0 & \sigma_{q_m}^2 & 0 & 0\\
0 & 0 & \sigma_{q_m}^2 & 0\\
0 & 0 & 0 & \sigma_{q_m}^2
\end{matrix}\right]
$$

#### Expected Covariance
$\boldsymbol{P}_{vv}$ is the expected covariance which is the sum of the projected state vector covariance $P_{zz}$ (aka uncertainty in the measurement caused by the uncertainty in the state vector prediction) and the measurement noise covariance $\boldsymbol{R}$.

$$
\boldsymbol{P}_{vv} = \boldsymbol{P}_{zz} + \boldsymbol{R}
$$

$$
\boldsymbol{P}_{zz} = {1\over2n} \sum_{i=1}^{2n}[\mathcal{Z}_i-z_k^-][\mathcal{Z}_i - z_k^-]^T
$$

#### Cross Correlation Matrix
The cross correlation matrix $\boldsymbol{P}_{xz}$ relates the noise in the state vector to the noise in the measurement.

$$
\boldsymbol{P}_{xz} = {1\over2n} \sum_{i=1}^{2n}[\mathcal{W}_i'][\mathcal{Z}_i - z_k^-]^T
$$

### Kalman Gain Matrix

$$
\boldsymbol{K}_k = \left[ \begin{matrix}
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



<p align="center">
Kalman Gain
</p>

$$
\boldsymbol{K}_k = \boldsymbol{P}_{xz}\boldsymbol{P}_{vv}^{-1}
$$


<p align="center">
Update State Estimate
</p>

$$
\hat{x}_{k} = \hat{x}_{k}^- + \boldsymbol{K}_{k}v_k
$$

Where $\hat{x}_{k}$ is the a *poseriori estimate*, $\hat{x}_{k}^- $ is the *a priori estimate*, $\boldsymbol{K}_{k}$ is the kalman gain, and  $v_k$ is the *innovation*.

<p align="center">
Update Estimate Uncertainty (Covariance)
</p>

$$
\boldsymbol{P}_k = \boldsymbol{P}_k^- - \boldsymbol{K}_k\boldsymbol{P}_{vv}\boldsymbol{K}_k^T
$$
