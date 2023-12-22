import numpy as np
import matplotlib.pyplot as plt

from kf import KF



plt.figure()

DT = 0.1
NUM_STEPS = 1000
MEAS_EVERY_STEPS = 10


kf = KF(5.0, 10.0, 15.0, 0.5)


real_a = 3
real_v = 3
real_h = 3
meas_variance = 0.5**2


mus = []
covs = []

real_hs = []
real_vs = []
real_as = []


noisy_as = []

for step in range(NUM_STEPS):
    mus.append(kf.x)
    covs.append(kf.P)
    kf.predict(DT)

    real_v = real_v + DT * real_a
    real_h = real_h + DT * real_v

    real_hs.append(real_h)
    real_vs.append(real_v)
    real_as.append(real_a)

    noisy_a = real_a + np.random.randn()*np.sqrt(meas_variance)
    noisy_as.append(noisy_a)

    if step != 0 and step % MEAS_EVERY_STEPS == 0:
        kf.update(noisy_a, meas_variance)
    
    if step > 750:
        real_a += 0.02





plt.subplot(3, 1, 1)
plt.title("accel")
plt.plot(noisy_as, 'c', label='measured')
plt.plot([mu[0] for mu in mus], 'r', label='filterred')
plt.plot(real_as, 'b', label='real')
plt.legend()
# plt.plot([mu[0] - 2*np.sqrt(cov[0, 0]) for mu, cov in zip(mus, covs)], 'r--') # lower bound
# plt.plot([mu[0] + 2*np.sqrt(cov[0, 0]) for mu, cov in zip(mus, covs)], 'r--') # upper bound

plt.subplot(3, 1, 2)
plt.title("vel")
plt.plot([mu[1] for mu in mus], 'r')
plt.plot(real_vs, 'b')
# plt.plot([mu[1] + 2*np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], 'r--') # lower bound
# plt.plot([mu[1] - 2*np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], 'r--') # upper bound

plt.subplot(3, 1, 3)
plt.title("height")
plt.plot([mu[2] for mu in mus], 'r')
plt.plot(real_hs, 'b')
# plt.plot([mu[1] + 2*np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], 'r--') # lower bound
# plt.plot([mu[1] - 2*np.sqrt(cov[1, 1]) for mu, cov in zip(mus, covs)], 'r--') # upper bound


plt.show()
