import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.

    # omega = dth/dt => omega*dt = dth
    v_t,omega_t = u
    x_t,y_t,theta_t = xvec

    if abs(omega_t) < EPSILON_OMEGA:
        theta_t1 = theta_t + omega_t*dt
        x_t1 = x_t + v_t*np.cos(theta_t1)*dt
        y_t1 = y_t + v_t*np.sin(theta_t1)*dt
        g = np.array([x_t1,y_t1,theta_t1])
    else:
        theta_t1 = theta_t + omega_t*dt
        x_t1 = x_t + v_t/omega_t*(np.sin(theta_t1)-np.sin(theta_t))
        y_t1 = y_t + v_t/omega_t*(-np.cos(theta_t1)+np.cos(theta_t))
        g = np.array([x_t1,y_t1,theta_t1])

    if compute_jacobians:
        if abs(omega_t) < EPSILON_OMEGA:
            Gx = np.array([[1 , 0 , -v_t*np.sin(theta_t1)*dt],
                           [0 , 1 ,  v_t*np.cos(theta_t1)*dt],
                           [0 , 0 , 1]])
            Gu = np.array([[np.cos(theta_t1)*dt , -0.5*v_t*np.sin(theta_t1)*dt**2],
                           [np.sin(theta_t1)*dt ,  0.5*v_t*np.cos(theta_t1)*dt**2],
                           [0 , dt]])
        else:
            Gx = np.array([[1 , 0 , v_t/omega_t*(np.cos(theta_t1)-np.cos(theta_t))],
                           [0 , 1 , v_t/omega_t*(np.sin(theta_t1)-np.sin(theta_t))],
                           [0 , 0 , 1]])
            Gu = np.array([[( np.sin(theta_t1)-np.sin(theta_t))/omega_t , v_t/(omega_t**2)*(np.sin(theta_t)-np.sin(theta_t1))+v_t*dt/omega_t*np.cos(theta_t1)],
                           [(-np.cos(theta_t1)+np.cos(theta_t))/omega_t , v_t/(omega_t**2)*(np.cos(theta_t1)-np.cos(theta_t))+v_t*dt/omega_t*np.sin(theta_t1)],
                           [0 , dt]])

    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def trans_line_a2b(line,a2b):
    # line is line expression in old frame
    # a2b is origin of new frame expressed in old frame

    alpha,r   = line
    x,y,theta = a2b

    u = np.array([x,y])                           # Vector of origin to new frame origin in old frame
    v = np.array([np.cos(alpha),np.sin(alpha)])   # Unit vector of origin to line in old frame (therefore no r)

    new_alpha = alpha - theta
    new_r     = r - np.dot(u,v)     # Since v is unit, no need to divide by |v|
    return np.array([new_alpha,new_r])

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)

    h = trans_line_a2b( trans_line_a2b( line , x ) , tf_base_to_camera )     # line -> line in base -> line in camera

    if compute_jacobian:
        xc,yc,thetac = tf_base_to_camera
        xb,yb,thetab = x
        alphao,ro    = line

        drc_dtheta = yc*np.cos(alphao-thetab) - xc*np.sin(alphao-thetab)
        Hx = np.array([[0,0,-1],
                       [-np.cos(alpha),-np.sin(alpha),drc_dtheta]])

    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h

