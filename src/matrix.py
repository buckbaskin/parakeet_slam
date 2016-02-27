import numpy as np

from numpy import dot as mm
from viz_feature_sim.msg import VizScan, Blob

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

def blob_to_matrix(blob):
    if isinstance(blob, Blob):
        return np.array([blob.bearing, blob.color.r, blob.color.g, blob.color.b])
    else:
        return blob