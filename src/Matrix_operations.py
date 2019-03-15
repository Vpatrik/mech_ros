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

covariances = np.zeros([5,3,3])
min_covariance = 1e-4

# covariances[0] = np.array([[0,1,2],[3,4,5],[6,7,8]])

mean = np.zeros([5,1,3])
# print(mean)
# print(covariances[:])
# print(mean.shape[0])
# Cov_matrix = np.zeros([3,3])
# print(Cov_matrix)

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

covariances = np.random.rand(3,3,3)
mean = np.random.rand(3,3,1)

# print(mean)
# print("--------------------")
# mean = mean[:-1]
# print(mean)

# print(covariances[0])
# mean[0] = np.array((8))
# mean[1] = np.array((2))

# covariances[0] = np.array((0.5))
# covariances[1] = np.array((0.5))

# cov_meas, cov_trans = calculateCovariances(20000, 2)
# print(cov_meas)
# print(cov_trans)

# mean_n, cov_n = multiVariateGaussianProduct(mean, covariances)

# print(mean_n[2])
# print(cov_n)

# cov = np.random.rand(3,3)

# tr_cov = calculateTransformedCovariance(5,2,1.57,3,4,3.15, cov_meas, cov_trans)
# tr_cov[0,2] = 1
# print(tr_cov)

# print(tr_cov.dot(np.eye(3)).T)
# print(tr_cov[:2,:2])

# pok = np.zeros((1,3,3))
# print(pok[0])


# a = []
# b = [1,2,3]
# c = [0,0,0]
# d = [7,8,9]
# e = [1,20,1]
# a.append(b)
# a.append(c)
# a.append(d)
# a.append(e)

# # # m = np.matrix(a)
# n = np.array(a)

# n*=10
# print(n)
# # print(m)
# print(n)
# cov_imu = np.cov(n,rowvar = False)
# print(cov_imu)

# mean = np.mean(n, axis = 0)
# print(mean)


# s = np.zeros([3,1])
# print(s)

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

num = 3
A = np.random.rand(num,3,3)
covariances = np.zeros([num,3,3])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)

mean = np.random.rand(num,3,1)

# print(covariances)

# m, c = non_inf_mult_gauss(mean,covariances)
# u,v = multiVariateGaussianProduct(mean, covariances)

# print(m)
# print(c)

# timeit.timeit(stmt='pass', setup='pass', timer=<default timer>, number=1000000, globals=None)
# print(covariances)
# setup = "import numpy as np"
# mean = np.random.rand(num,3,1)

def inf():
    SETUP_CODE = '''
from __main__ import non_inf_mult_gauss
import numpy as np
from scipy import linalg as lin
    '''

    TEST_CODE = '''
num = 4
s = 3
A = np.random.rand(num,s,s)
covariances = np.zeros([num,s,s])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)
mean = np.random.rand(num,s,1)
non_inf_mult_gauss(mean,covariances)
'''
    # timeit.repeat statement 
    times = timeit.repeat(setup = SETUP_CODE, 
                          stmt = TEST_CODE, 
                          repeat = 1, 
                          number = 100000)
    # priniting minimum exec. time 
    print('Cholesky: {}'.format(min(times)))


def chol():
    SETUP_CODE = '''
from __main__ import multiVariateGaussianProduct
import numpy as np
    '''

    TEST_CODE = '''
num = 4
s = 3
A = np.random.rand(num,s,s)
covariances = np.zeros([num,s,s])
for i in range(num):
    covariances[i] = np.dot(A[i],A[i].T)
mean = np.random.rand(num,s,1)
multiVariateGaussianProduct(mean,covariances)
'''
    # timeit.repeat statement 
    times = timeit.repeat(setup = SETUP_CODE, 
                          stmt = TEST_CODE, 
                          repeat = 1, 
                          number = 100000)
    # priniting minimum exec. time 
    print('Informative: {}'.format(min(times)))

# mean1, cov1 = non_inf_mult_gauss(mean,covariances)
# mean2, cov2 = multiVariateGaussianProduct(mean,covariances)

# print(mean1)
# print(cov1)
# print('-----------')
# print(mean2)
# print(cov2)

# print(timeit.timeit(stmt =Test_code,setup=setup, number=100))


# if __name__ == "__main__":
#     chol()
#     inf()

pok = np.random.rand(3,3,3)
# pok = np.ones((3,3,3))
print(pok[0])
print(pok[0,:2,:2])
print(pok[0,2,2])
ful = pok[:,0,0]
inv = 1/ful
print(ful)
print(inv)
print(sum(inv))
print(sum(1/pok[:,0,0]))
ful_sum = np.sum(pok[:,0,0])
print(ful_sum)

print(np.sum(pok, axis = 0)/pok.shape[0])
if pok.size == 0:
    print(pok)

print(pok[1,0,0])
nul = np.ones((2,3,3))
nul[1,1,:] = 2,2,2
nul2 = np.zeros((2,3,3))
res = nul - nul2
sum_nul = sum(nul)
avg_nul = np.mean(nul, axis= 0)
print(abs(sum_nul))
print(avg_nul)
# print(nul.size)
# print(nul.shape)
# print(nul)
# print(pok)