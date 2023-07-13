import numpy as np


def lse(offsets, measurements, limitx = [-1,8], limity = [-1,8], limitz = [0 , 1.5]):
    '''
        Simple least squares approach for (up to) 4 anchors on the UGv

        Measurements is a list of length num_of_anchors
    '''

    min_err = None
    min_err_pos = [0, 0]

    xys = np.mgrid[limitx[0]:limitx[1]:0.01, limity[0]:limity[1]:0.01, limitz[0]:limitz[1]:0.1].reshape(3, -1).T

    err = None
    for idx, m in enumerate(measurements) :
        if err is None :
            err = ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
        else :
            err += ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
    # err = ( measurements - numpy.linalg.norm(xys - offsets[idx], axis=1) ) ** 2

    min_err_idx = np.argmin(err)
    min_err = err[min_err_idx]
    min_err_pos = xys[min_err_idx]
    # print(min_err_pos, min_err)
    return min_err_pos, min_err