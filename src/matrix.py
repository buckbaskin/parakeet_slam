import numpy as np

class Matrix(np.ndarray):
    def __init__(self, array_like):
        self = np.array(array_like)

    def __add__(self, right):
        madd(self, right)
    def __radd__(self, left):
        madd(left, self)

def Matrix(array_like):
    # TODO(buckbaskin): make it into the 2D numpy-array
    return np.array(array_like)

def inverse(matrix):
    return np.linalg.inv(matrix)

def transpose(matrix):
    return np.transpose(matrix)

def mmultiply(left, right):
    return np.dot(left, right)

def madd(left, right):
    return np.add(left, right)

def msubtract(left, right):
    # TODO(buckbaskin):
    return np.subtract(left, right)

def identity(n):
    # TODO(buckbaskin):
    return np.identity(n)