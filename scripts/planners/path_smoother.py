import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    from scipy.interpolate import splrep, splev
    path = np.array(path)
    tseg = np.linalg.norm(np.diff(path,axis=0),axis=1)/V_des
    t    = np.insert(np.cumsum(tseg),0,0)
    t_sm = np.arange(0,t[-1],dt)

    splx = splrep(t,path[:,0],s=alpha)
    sply = splrep(t,path[:,1],s=alpha)
    
    x    = splev(t_sm, splx)
    x_d  = splev(t_sm, splx, der=1)
    x_dd = splev(t_sm, splx, der=2)

    y    = splev(t_sm, sply)
    y_d  = splev(t_sm, sply, der=1)
    y_dd = splev(t_sm, sply, der=2)

    th   = np.arctan2(y_d,x_d)

    traj_smoothed = np.column_stack([x,y,th,x_d,y_d,x_dd,y_dd])
    t_smoothed    = t_sm
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
