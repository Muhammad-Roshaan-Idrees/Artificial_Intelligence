"""Muhammad Roshaan Idrees_56177_AI Lab2

**Muhammad Roshaan Idrees
56177**

**List Methods in Python**

**Append a list**
"""

my_list=[3,1,4,1,5]
my_list.append(9)
print(my_list)

"""**Clear a list**"""

my_list.clear()
print(my_list)

"""**Copy a list**"""

my_list=[3,1,4,1,5]
copy_list=my_list.copy()
print(copy_list)

"""**Count the specific element**"""

print(my_list.count(1))

"""**Add elements to end of list**"""

my_list=[3,1,4,1,5]
my_list.extend([10,11])
print(my_list)

"""**Index of first element with specified value**"""

print(my_list.index(4))

"""**Insert element at specific index**"""

my_list.insert(2,8)
print(my_list)

"""**Removes an element at specified position**"""

my_list.pop(3)
print(my_list)

"""**Remove the item with specified value**




"""

my_list.remove(5)
print(my_list)

"""**Reverse the order of list**"""

my_list.reverse()
print(my_list)

"""**Sort the list**"""

my_list.sort()
print(my_list)