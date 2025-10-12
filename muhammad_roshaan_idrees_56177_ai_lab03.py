"""Muhammad Roshaan Idrees_56177_AI_Lab03

**Muhammad Roshaan Idrees
56177**

**Maximum and Minimum value of flattened array**
"""

import numpy as np

#Creating and arranging the input into 3x3 matrix
matrix = np.arange(2,11).reshape(3,3)

#Converting the values of matrix in float
float_matrix = matrix.astype(float)

#Flattening the array
flattened_array = float_matrix.flatten()

#Finding Maximum and Minimum values of Flattened array
max_value = np.max(flattened_array)
min_value = np.min(flattened_array)

#Printing the result
print("Original Matrix:", float_matrix)
print("Flattened Array:", flattened_array)
print("Maximum value:", max_value)
print("Minimum value:", min_value)