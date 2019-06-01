import numpy as np
from src.Estimated_pose_markers import inversion, PoseEstimator
from src.Matrix_operations import non_inf_mult_gauss, multiVariateGaussianProductHardCoded

def test_matrixInversion():
    M = np.random.rand(3,3)
    manual = np.around(inversion(M), 10)
    inv_num = np.around(np.linalg.inv(M),10)
    assert np.array_equal(manual, inv_num)


def test_limitMatrix():
    M = np.array([[-0.01,-0.1,0.01],[-0.01,-0.1,0.01],[-0.01,-0.1,0.01]])
    pose = PoseEstimator()
    pose.min_covariance = 0.05
    M_limit = np.array([[-0.05,-0.1,0.05],[-0.05,-0.1,0.05],[-0.05,-0.1,0.05]])
    M_limit_method = pose.limitMatrix(M)
    assert np.array_equal(M_limit, M_limit_method)

def test_multGaussFusion():
    num = 4
    s = 3
    min_covariance = 1e-4
    A = np.random.rand(num,s,s)
    covariances = np.zeros([num,s,s])
    for i in range(num):
        covariances[i] = np.dot(A[i],A[i].T)
    covariances[covariances < min_covariance] = min_covariance

    mean = np.random.rand(num,s,1)

    informative = np.around(non_inf_mult_gauss(mean, covariances)[0],10)
    canonical = np.around(multiVariateGaussianProductHardCoded(mean, covariances)[0],10)

    assert np.array_equal(informative, canonical)

if __name__ == "__main__":
    test_matrixInversion()
    test_limitMatrix()
    test_multGaussFusion()