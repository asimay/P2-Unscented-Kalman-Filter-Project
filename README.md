## **Unscented Kalman Filter Project**

The goals / steps of this project are the following:

* Utilize Unscented kalman filter and  based on the CTRV motion model to estimate the state of a moving object of interest with noisy lidar and radar measurements. and final RMSE <= [.09, .10, .40, .30]. This proved the CTRV model is more precise than the constant velocity model from the EKF. And UKF is also known for handling non-linear equations better than EKF.

[//]: # (Image References)
[image1]: ./outputs/1.png
[image2]: ./outputs/onlyradar.png
[image3]: ./outputs/onlylaser.png
[image4]: ./outputs/dataset2.png
[image5]: ./outputs/final.png

Final result:

![alt text][image5]

---

## UKF Process


#### 1. Read the Radar and Lidar measurement data into program.

This is provided in main.cpp file.

#### 2. Unscented Kalman Filter Process:

##### 1. Generating Sigma Points, which is used to estimate and replace the non-linear process noise model.

`X​k∣k​​ =[x​k∣k​​ x​k∣k​​ +√​(λ+n​x​​ )P​k∣k​​ x​k∣k​​ −√​(λ+n​x​ )P​k∣k​​ ]`

##### 2. Using augmented sigma points, which involved process noise on state vector x.

`Augmented State = x​a,k​​ =​[​p​x ​p​y ​v ​ψ ​ψ​˙ ​ν​a ​ν​ψ​¨​].transpose()`

##### 3. Predict the sigma points.

`State = x​k+1​​ = x​k​​ + g(k) + nu(k)`

##### 4. Predict the mean and covariance.

Predicted Mean:

`x​k+1∣k​​ =∑​​ w​i​​ * X​k+1∣k,i`
​​
Predicted Covariance:

`P​k+1∣k​​ =∑​w​i​*​ (X​k+1∣k,i​​ −x​k+1∣k​​ )(X​k+1∣k,i​​ −x​k+1∣k​​ )​T`
​​

##### 5. Predict measurement:

Measurement Model:

`z​k+1∣k​​ =h(x​k+1​​ )+w​k+1`
​​
##### 6. Update state:

Update State

`x​k+1∣k+1​​ =x​k+1∣k​​ +K​k+1∣k​​ (z​k+1​​ −z​k+1∣k​​ )`

##### 7. Normalizing Angles:

```
	VectorXd T =  Zsig.col(i) - z_pred;
	while(T(1) < -M_PI) T(1) += 2*M_PI;
	while(T(1) > M_PI)  T(1) -= 2*M_PI;
```

## Test Implementation:


#### 1. Fusion both Radar and Lidar data:

RMSE screenshot as below:

![alt text][image1]

#### 2. Fusion only Radar data:

RMSE screenshot as below:

![alt text][image2]

#### 3. Fusion only Lidar data:

RMSE screenshot as below:

![alt text][image3]

#### 4. Dataset 2 test with Lidar and Radar data:

![alt text][image4]

####  5. Fusion both Radar and Lidar data, but tune a good Q process noise, I increase the std_a_ = 1.885 and std_yawdd_ = 0.28, and got a good result for combined with both dataset 1 and 2:

RMSE screenshot as below:

![alt text][image5]



