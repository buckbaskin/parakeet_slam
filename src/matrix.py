import numpy as np

from numpy import dot as mm

# class Matrix(object):
#     def __init__(self, array_like):
#         print 'Matrix __init__ ...'
#         self = np.array(array_like)
#         print 'end __init__'

    # def __add__(self, right):
    #     madd(self, right)
    # def __radd__(self, left):
    #     madd(left, self)

    # def __sub__(self, right):
    #     msubtract(self, right)
    # def __rsub__(self, left):
    #     msubtract(left, self)

    # def __mul__(self, right):
    #     print '__mul__ override'
    #     return np.dot(self, right)
    # def __rmul__(self, left):
    #     print '__rmul__ override'
    #     return np.dot(left, self)

    # def __xor__(self, power):
    #     print('xor called')
    #     if power == -1:
    #         return inverse(self)
    #     else:
    #         return self

def Matrix(array_like):
    print 'Matrix method'
    np_array = np.array(array_like)
    return np_array

def inverse(matrix):
    return np.linalg.inv(matrix)

def transpose(matrix):
    return np.transpose(matrix)

def mmultiply(left, right):
    return mm(left, right)

def madd(left, right):
    return np.add(left, right)

def msubtract(left, right):
    # TODO(buckbaskin):
    return np.subtract(left, right)

def identity(n):
    # TODO(buckbaskin):
    return np.identity(n)

def magnitude(matrix):
    pass