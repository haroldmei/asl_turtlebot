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
    from scipy.interpolate import splev, splrep
    sz = len(path)
    extended_ts = [0.0]
    ts = [0.0]
    for i in range(1, sz):
        l = np.linalg.norm(np.array(path[i-1]) - np.array(path[i])) * 1.2
        tm = l/V_des
        extended_ts = extended_ts + list(np.arange(extended_ts[-1], extended_ts[-1] + tm, dt))
        ts.append(ts[-1] + tm)

    sa = splrep(ts, np.array(path)[:,0])
    sb = splrep(ts, np.array(path)[:,1])

    traj_smoothed = np.zeros((len(extended_ts), 7))
    traj_smoothed[:,0] = splev(extended_ts, sa)
    traj_smoothed[:,1] = splev(extended_ts, sb)
    traj_smoothed[:,3] = splev(extended_ts, sa, der=1)
    traj_smoothed[:,4] = splev(extended_ts, sb, der=1)    
    traj_smoothed[:,5] = splev(extended_ts, sa, der=2)
    traj_smoothed[:,6] = splev(extended_ts, sb, der=2)

    traj_smoothed[:,2] = np.arccos(traj_smoothed[:,3]/(traj_smoothed[:,3]**2 + traj_smoothed[:,4]**2)**0.5) 

    t_smoothed = extended_ts
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
