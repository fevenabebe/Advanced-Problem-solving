import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad, dblquad  # For exact value comparison

# Function to integrate: f(x) = e^(-x^2)
def f_single(x):
    return np.exp(-x**2)

# Monte Carlo integration for a single variable
def monte_carlo_integration_single(a, b, num_samples):
    samples = np.random.uniform(a, b, num_samples)  # Random samples in [a, b]
    integral_estimate = (b - a) * np.mean(f_single(samples))  # Estimate integral
    return integral_estimate

# Function to integrate in 2D: f(x, y) = e^(-(x^2 + y^2))
def f_double(x, y):
    return np.exp(-(x**2 + y**2))

# Monte Carlo integration for two variables
def monte_carlo_integration_double(a, b, num_samples):
    x_samples = np.random.uniform(a, b, num_samples)  # Random x samples
    y_samples = np.random.uniform(a, b, num_samples)  # Random y samples
    integral_estimate = (b - a) ** 2 * np.mean(f_double(x_samples, y_samples))  # Estimate integral
    return integral_estimate

# Set integration limits
a, b = 0, 1

# Sample sizes to test
sample_sizes = [100, 1000, 10000, 100000]
single_integral_estimates = []
double_integral_estimates = []

# Exact value for comparison using numerical integration
exact_single = quad(f_single, a, b)[0]
exact_double, _ = dblquad(f_double, a, b, lambda x: a, lambda x: b)  # Use dblquad for 2D

# Loop over different sample sizes for single variable
for N in sample_sizes:
    estimate = monte_carlo_integration_single(a, b, N)
    single_integral_estimates.append(estimate)

# Loop over different sample sizes for double variable
for N in sample_sizes:
    estimate = monte_carlo_integration_double(a, b, N)
    double_integral_estimates.append(estimate)

# Plotting the results
plt.figure(figsize=(12, 6))

# Single variable results
plt.subplot(1, 2, 1)
plt.plot(sample_sizes, single_integral_estimates, marker='o', label='Estimated Integral')
plt.axhline(y=exact_single, color='r', linestyle='--', label='Exact Integral')
plt.xscale('log')
plt.xlabel('Number of Samples (N)')
plt.ylabel('Integral Estimate')
plt.title('Monte Carlo Integration: Single Variable')
plt.legend()

# Double variable results
plt.subplot(1, 2, 2)
plt.plot(sample_sizes, double_integral_estimates, marker='o', label='Estimated Integral (2D)')
plt.axhline(y=exact_double, color='r', linestyle='--', label='Exact Integral (2D)')
plt.xscale('log')
plt.xlabel('Number of Samples (N)')
plt.ylabel('Integral Estimate (2D)')
plt.title('Monte Carlo Integration: Double Variable')
plt.legend()

# Show the plots
plt.tight_layout()
plt.show()