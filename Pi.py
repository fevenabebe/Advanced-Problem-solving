import numpy as np
import matplotlib.pyplot as plt
# Function to estimate using the Monte Carlo method
def estimate_pi(num_samples):
inside_circle = 0 # Count of points inside the circle
for _ in range(num_samples): # Loop for the number of samples
# Generate random x and y coordinates between 0 and 1
x = np.random.rand()
y = np.random.rand()
# Check if the point is inside the unit circle
if x**2 + y**2 <= 1:
inside_circle += 1 # Increment count if inside the circle
# Calculate estimate
pi_estimate = 4 * inside_circle / num_samples
return pi_estimate # Return the estimated value of
# List of sample sizes to test
sample_sizes = [1000, 10000, 100000, 1000000]
pi_estimates = [] # To store estimates
errors = [] # To store errors
# True value of for comparison
pi_actual = np.pi
# Loop over different sample sizes
for N in sample_sizes:
pi_estimate = estimate_pi(N) # Estimate
pi_estimates.append(pi_estimate) # Store the estimate
# Calculate the error
error = abs(pi_estimate - pi_actual)
errors.append(error) # Store the error
# Plotting the results
plt.figure(figsize=(12, 6))
# Plot for estimated values
plt.subplot(1, 2, 1)
plt.plot(sample_sizes, pi_estimates, marker='o', label='Estimated ')
plt.axhline(y=pi_actual, color='r', linestyle='--', label='Actual ')
plt.xscale('log') # Use logarithmic scale for x-axis
plt.xlabel('Number of Samples (N)')
1
plt.ylabel('Estimated ')
plt.title('Convergence of Estimation')
plt.legend()
# Plot for errors
plt.subplot(1, 2, 2)
plt.plot(sample_sizes, errors, marker='o', color='orange', label='Error')
plt.xscale('log') # Use logarithmic scale for x-axis
plt.yscale('log') # Use logarithmic scale for y-axis
plt.xlabel('Number of Samples (N)')
plt.ylabel('Error | estimated - actual|')
plt.title('Error in Estimation')
plt.grid()
# Show the plots
plt.tight_layout()
plt.show()