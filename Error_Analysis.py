import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad, dblquad

# Function to integrate: f(x) = e^(-x^2)
def f_single(x):
    return np.exp(-x**2)

# Monte Carlo integration for a single variable
def monte_carlo_integration_single(a, b, num_samples):
    samples = np.random.uniform(a, b, num_samples)
    function_values = f_single(samples)
    integral_estimate = (b - a) * np.mean(function_values)
    std_dev = np.std(function_values)
    error = abs(integral_estimate - quad(f_single, a, b)[0])
    return integral_estimate, error, std_dev

# Function to integrate in 2D: f(x, y) = e^(-(x^2 + y^2))
def f_double(x, y):
    return np.exp(-(x**2 + y**2))

# Monte Carlo integration for two variables
def monte_carlo_integration_double(a, b, num_samples):
    x_samples = np.random.uniform(a, b, num_samples)
    y_samples = np.random.uniform(a, b, num_samples)
    function_values = f_double(x_samples, y_samples)
    integral_estimate = (b - a)**2 * np.mean(function_values)
    std_dev = np.std(function_values)
    error = abs(integral_estimate - dblquad(f_double, a, b, lambda x: a, lambda x: b)[0])
    return integral_estimate, error, std_dev

# Set integration limits
a, b = 0, 1

# Sample sizes to test
sample_sizes = [100, 1000, 10000, 100000]
single_errors = []
double_errors = []

# Loop over different sample sizes for single variable
for N in sample_sizes:
    _, error, _ = monte_carlo_integration_single(a, b, N)
    single_errors.append(error)

# Loop over different sample sizes for double variable
for N in sample_sizes:
    _, error, _ = monte_carlo_integration_double(a, b, N)
    double_errors.append(error)

# Theoretical error bounds for comparison
theoretical_errors_single = [np.std(np.exp(-np.random.uniform(a, b, N)**2)) / np.sqrt(N) for N in sample_sizes]
theoretical_errors_double = [np.std(np.exp(-np.random.uniform(a, b, N)**2)) / np.sqrt(N) for N in sample_sizes]

# Plotting the errors
plt.figure(figsize=(12, 6))

# Single variable error plot
plt.subplot(1, 2, 1)
plt.plot(sample_sizes, single_errors, marker='o', label='Monte Carlo Error (Single)')
plt.plot(sample_sizes, theoretical_errors_single, marker='x', label='Theoretical Error (Single)', linestyle='--')
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Number of Samples (N)')
plt.ylabel('Error')
plt.title('Error Analysis: Single Variable')
plt.legend()

# Double variable error plot
plt.subplot(1, 2, 2)
plt.plot(sample_sizes, double_errors, marker='o', label='Monte Carlo Error (2D)')
plt.plot(sample_sizes, theoretical_errors_double, marker='x', label='Theoretical Error (2D)', linestyle='--')
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Number of Samples (N)')
plt.ylabel('Error')
plt.title('Error Analysis: Double Variable')
plt.legend()

# Show the plots
plt.tight_layout()
plt.show()