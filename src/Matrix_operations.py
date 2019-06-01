#!/usr/bin/env python

# Patrik Vavra 2019

"""
Comparsion of different approaches to compute PoseWithCovariance from ArUco measurement
"""

import numpy as np
import math
import timeit
from scipy import linalg as lin

K1 = 10
A1 = 1.1
K2 = 8
A2 = 1.1
K3 = 5
A3 = 1.1
alfa = 0.1
min_covariance = 1e-4

# Compute product for n gaussians
def multiVariateGaussianProduct(Mean_vectors, Cov_matrices):

    # Initialize resulting matrices
    Cov_matrix = Cov_matrices[0]
    Mean_vector = Mean_vectors[0]

    matrices_num = Mean_vectors.shape[0]
    variate_num = Mean_vectors.shape[1]
    
    if matrices_num < 2:
        return Mean_vector, Cov_matrix

    # Initialize information matrices
    inf_vec = np.zeros([variate_num,1])
    inf_mat = np.zeros([variate_num,variate_num])
    inf_mat_sum = np.zeros([variate_num,variate_num])

    # for i in range(Mean_vectors.shape[0]-1):
    #     Cov_matrices[i] = np.dot(np.dot(Cov_matrix, np.linalg.inv(Cov_matrix + Cov_matrices[i+1])), Cov_matrices[i+1])
    #     Mean_vectors[i] = np.dot(np.dot(Cov_matrices[i+1], np.linalg.inv(Cov_matrix + Cov_matrices[i+1])), Mean_vector) + 
    #                         np.dot(np.dot(Cov_matrices[i+1], np.linalg.inv(Cov_matrix + Cov_matrices[i+1])), Mean_vectors[i+1])
    #     Cov_matrix = Cov_matrices[i]
    #     Mean_vector = Mean_vectors[i]

    # Compute information matrix and information vector and then convert to mean vector and covariance matrix

    for i in range(matrices_num):
        inf_mat = np.linalg.inv(Cov_matrices[i])
        inf_mat_sum += inf_mat
        inf_vec += np.dot(inf_mat, Mean_vectors[i])

    Cov_matrix = np.linalg.inv(inf_mat_sum)
    Mean_vector = np.dot(Cov_matrix, inf_vec)

    return Mean_vector, Cov_matrix

def inversion(m):    
    m1, m2, m3, m4, m5, m6, m7, m8, m9 = m.ravel()
    inv = np.array([[m5*m9-m6*m8, m3*m8-m2*m9, m2*m6-m3*m5],
                    [m6*m7-m4*m9, m1*m9-m3*m7, m3*m4-m1*m6],
                    [m4*m8-m5*m7, m2*m7-m1*m8, m1*m5-m2*m4]])
    return inv / np.dot(inv[0], m[:, 0])

# Compute product for n gaussians
def multiVariateGaussianProductHardCoded(Mean_vectors, Cov_matrices):

    # Initialize resulting matrices
    Cov_matrix = Cov_matrices[0]
    Mean_vector = Mean_vectors[0]

    matrices_num = Mean_vectors.shape[0]
    variate_num = Mean_vectors.shape[1]
    
    if matrices_num < 2:
        return Mean_vector, Cov_matrix

    # Initialize information matrices
    inf_vec = np.zeros([variate_num,1])
    inf_mat = np.zeros([variate_num,variate_num])
    inf_mat_sum = np.zeros([variate_num,variate_num])

    # Compute information matrix and information vector and then convert to mean vector and covariance matrix
    for i in range(matrices_num):
        inf_mat = inversion(Cov_matrices[i])
        inf_mat_sum += inf_mat
        inf_vec += np.dot(inf_mat, Mean_vectors[i])

    Cov_matrix = inversion(inf_mat_sum)
    Mean_vector = np.dot(Cov_matrix, inf_vec)

    return Mean_vector, Cov_matrix

def calculateTransformedCovariance2(x, y, fi, a, b, Fi, covarianceMatrix):
    ## Parameters
    # x,y translation from marker to camera
    # fi rotation from marker to camera
    # a,b, translation from global frame to marker
    # Fi rotation from global frame to camera

    # Compute transformed covariance from covariance and transformation matrix
    # transformedCovariance = np.zeros([3,3])
    # transformedCovariance[0][0] = math.cos(fi+Fi)
    # transformedCovariance[0][1] = - math.sin(fi+Fi)
    # transformedCovariance[0][2] = a/(fi+Fi)
    # transformedCovariance[1][0] = math.sin(fi+Fi)
    # transformedCovariance[1][1] = math.cos(fi+Fi)
    # transformedCovariance[1][2] = b/(fi+Fi)
    # transformedCovariance[2][2] = 1

    # transformedCovariance = np.dot(np.dot(transformedCovariance, covarianceMatrix), transformedCovariance.T)

    # Only compute each member of transformed covariance from analytical solution
    # covariance is represented as diagonal
    var_x = covarianceMatrix[0][0]
    var_y = covarianceMatrix[1][1]
    var_fi = covarianceMatrix[2][2]

    transformedCovariance = np.zeros([3,3])
    transformedCovariance[0][0] = (math.cos(fi+Fi))**2*var_x + (a/(fi+Fi))**2*var_fi + (math.sin(fi+Fi))**2*var_y
    transformedCovariance[0][1] = a*b*var_fi/((fi+Fi)**2) + math.cos(Fi+fi)*math.sin(Fi+fi)*var_x - math.cos(Fi+fi)*math.sin(Fi+fi)*var_y
    transformedCovariance[0][2] = a*var_fi/(fi+Fi)
    transformedCovariance[1][0] = transformedCovariance[0][1]
    transformedCovariance[1][1] = (b/(fi+Fi))**2*var_fi + (math.sin(fi+Fi))**2*var_x + (math.cos(fi+Fi))**2*var_y
    transformedCovariance[1][2] = b*var_fi/(fi+Fi)
    transformedCovariance[2][0] = transformedCovariance[0][2]
    transformedCovariance[2][1] = transformedCovariance[1][2]
    transformedCovariance[2][2] = var_fi


    return transformedCovariance

def CalculateCovariance2(S_rel, Yaw_rel):
    S_norm = S_rel/abs(math.cos(Yaw_rel))
    Covariance = np.zeros([3,3])
    Covariance[0][0] = K1/(S_norm**A1)
    Covariance[1][1] = K2/(S_norm**A2)
    Covariance[2][2] = K3/(S_norm**A3)
    return Covariance

def calculateCovariances(S_rel, Yaw_rel):
    S_norm = S_rel/max(abs(math.cos(Yaw_rel)),abs(math.sin(Yaw_rel)))    
    # S_norm = S_rel/abs(math.cos(Yaw_rel))
    covarianceMeas = np.zeros([2,2])
    covarianceMeas[0,0] = K1/(S_norm**A1)
    covarianceMeas[1,1] = K2/(S_norm**A2)
    covarianceMeas[covarianceMeas < min_covariance] = min_covariance

    # Constant covariance for a_x and a_y, Fi -  marker in global coordinate system
    # distance : 0.01**2, angle~: 0.0175**2 
    covarianceTrans = np.zeros([3,3])
    covarianceTrans[0,0] = 0.001
    covarianceTrans[1,1] = 0.001
    # covariance in angle = cov_fi + cov_Fi
    covarianceTrans[2,2] = K3/(S_norm**A3) + 0.0005
    if covarianceTrans[2,2] < min_covariance:
        covarianceTrans[2,2] = min_covariance

    return covarianceMeas, covarianceTrans


def calculateTransformedCovariance(a_x, a_y, fi, x, y, Fi, covarianceMeas, covarianceTrans):
    ############ TO DO ###############x
    
    ## Parameters
    # a_x,a_y translation from marker to camera in camera coordinate system
    # fi rotation from marker to camera
    # x,y, translation from global frame to marker
    # Fi rotation from global frame to camera

    # Compute Jacobians
    # J_a - Jacobian of point measurement
    J_a = np.zeros([2,2])
    J_a[0,0] = math.cos(fi+Fi)
    J_a[0,1] = -math.sin(fi+Fi)
    J_a[1,0] = math.sin(fi+Fi)
    J_a[1,1] = -math.sin(fi+Fi)

    # J_p - Jacobian of transformation
    J_p = np.zeros([2,3])
    J_p[0,0] = 1
    J_p[0,2] = -a_x*math.sin(fi+Fi) - a_y*math.cos(fi+Fi)
    J_p[1,1] = 1
    J_p[1,2] = a_x*math.cos(fi+Fi) - a_y*math.sin(fi+Fi)

    # Compute transformed covariance matrix of a'- Cov_a = J_a.covarianceMeas.J_a' + J_p.covarianceTrans.J_p'
    cov_a = np.dot(np.dot(J_a, covarianceMeas), J_a.T) + np.dot(np.dot(J_p, covarianceTrans), J_p.T)
    # print(cov_a)
    # Make 3x3 covariance matrix for transformed - a_x', a_y', theta  (theta = Fi + fi)
    transformedCovariance = np.zeros([3,3])
    transformedCovariance[:2,:2] = cov_a
    transformedCovariance[2,2] = covarianceTrans[2][2]

    return transformedCovariance

# Non information multivariate gaussian product
def non_inf_mult_gauss(Mean_vectors, Cov_matrices):

    # Initialize resulting matrices
    Cov_matrix = Cov_matrices[0]
    Mean_vector = Mean_vectors[0]

    matrices_num = Mean_vectors.shape[0]
    variate_num = Mean_vectors.shape[1]
    
    if matrices_num < 2:
        return Mean_vector, Cov_matrix


    for i in range(matrices_num-1):
        # Using scipy
        cholesky_matrix = lin.cholesky((Cov_matrix + Cov_matrices[i+1]), lower=True,check_finite=False)
        chol_cov1 = lin.solve_triangular(cholesky_matrix, Cov_matrix,lower=True,check_finite=False)
        chol_cov2 = lin.solve_triangular(cholesky_matrix, Cov_matrices[i+1],lower=True,check_finite=False)
        chol_mean1 = lin.solve_triangular(cholesky_matrix, Mean_vector,lower=True,check_finite=False)
        chol_mean2 = lin.solve_triangular(cholesky_matrix, Mean_vectors[i+1],lower=True,check_finite=False)

        # Using numpy
        # cholesky_matrix = np.linalg.cholesky(Cov_matrix + Cov_matrices[i+1])
        # chol_cov1 = np.linalg.solve(cholesky_matrix, Cov_matrix)
        # chol_cov2 = np.linalg.solve(cholesky_matrix, Cov_matrices[i+1])
        # chol_mean1 = np.linalg.solve(cholesky_matrix, Mean_vector)
        # chol_mean2 = np.linalg.solve(cholesky_matrix, Mean_vectors[i+1])


        Cov_matrix = np.dot(chol_cov1.T,chol_cov2)
        Mean_vector = np.dot(chol_cov2.T,chol_mean1) + np.dot(chol_cov1.T,chol_mean2)

    return Mean_vector, Cov_matrix

def limitMatrix(A, condition):
    m_abs = (abs(A) < condition)
    m_neg = (A < 0)
    m_pos = (A > 0)
    m_abs_neg = np.multiply(m_abs, m_neg)
    m_abs_pos = np.multiply(m_abs, m_pos)
    A[m_abs_neg] = -condition
    A[m_abs_pos] = condition
    return A

# timeit.timeit(stmt='pass', setup='pass', timer=<default timer>, number=1000000, globals=None)
# print(covariances)
# setup = "import numpy as np"
# mean = np.random.rand(num,3,1)



def chol():
    SETUP_CODE = '''
from __main__ import non_inf_mult_gauss
import numpy as np
from scipy import linalg as lin
    '''

    TEST_CODE = '''
num = 4
s = 3
min_covariance = 1e-4
A = np.random.rand(num,s,s)
covariances = np.zeros([num,s,s])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)
covariances[covariances < min_covariance] = min_covariance
mean = np.random.rand(num,s,1)
non_inf_mult_gauss(mean,covariances)
'''
    # timeit.repeat statement 
    times = timeit.repeat(setup = SETUP_CODE, 
                          stmt = TEST_CODE, 
                          repeat = 1, 
                          number = 100)
    # priniting minimum exec. time 
    print('Cholesky: {}'.format(min(times)))


def inf():
    SETUP_CODE = '''
from __main__ import multiVariateGaussianProduct
import numpy as np
    '''

    TEST_CODE = '''
num = 4
s = 3
min_covariance = 1e-4
A = np.random.rand(num,s,s)
covariances = np.zeros([num,s,s])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)
covariances[covariances < min_covariance] = min_covariance
mean = np.random.rand(num,s,1)
multiVariateGaussianProduct(mean,covariances)
'''
    # timeit.repeat statement 
    times = timeit.repeat(setup = SETUP_CODE, 
                          stmt = TEST_CODE, 
                          repeat = 1, 
                          number = 100)
    # priniting minimum exec. time 
    print('Informative: {}'.format(min(times)))


def infHardCoded():
    SETUP_CODE = '''
from __main__ import multiVariateGaussianProductHardCoded
import numpy as np
    '''

    TEST_CODE = '''
num = 4
s = 3
min_covariance = 1e-4
A = np.random.rand(num,s,s)
covariances = np.zeros([num,s,s])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)
covariances[covariances < min_covariance] = min_covariance

mean = np.random.rand(num,s,1)
multiVariateGaussianProductHardCoded(mean,covariances)
'''
    # timeit.repeat statement 
    times = timeit.repeat(setup = SETUP_CODE, 
                          stmt = TEST_CODE, 
                          repeat = 1, 
                          number = 100)
    # priniting minimum exec. time 
    print('Informative hard_coded: {}'.format(min(times)))


# print(timeit.timeit(stmt =Test_code,setup=setup, number=100))


# if __name__ == "__main__":
#     chol()
#     inf()
#     infHardCoded()

if __name__ == "__main__":

    # num = 4
    # s = 3
    # min_covariance = 1e-4
    # A = np.random.rand(num,s,s)
    # covariances = np.zeros([num,s,s])
    # for i in range(num):
    #     covariances[i] = np.dot(A[i],A[i].T)
    # covariances[covariances < min_covariance] = min_covariance

    # mean = np.random.rand(num,s,1)
    # mean1, cov1 = non_inf_mult_gauss(mean,covariances)
    # mean2, cov2 = multiVariateGaussianProduct(mean,covariances)

    # print(mean1)
    # print(cov1)
    # print('-----------')
    # print(mean2)
    # print(cov2)

    A = np.array([[-0.01,-0.1,0.01],[-0.01,-0.1,0.01],[-0.01,-0.1,0.01]])

    print(limitMatrix(A, 0.02))