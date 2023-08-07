import numpy as np

def Power(v):
    """Power consumption at speed v."""
    W = 20         # Weight
    p = 1.225      # Air density
    R = 0.4        # Rotor radius in meter
    A = 0.503      # Rotor disc area in meter^2
    omega = 300    # Blade angular velocity in radians/s
    Utip = 120     # Tip of the rotor blade (m/s)
    s = 0.05       # Rotor solidity
    d_0 = 0.6      # Fuselage drag ratio
    k = 0.1        # Incremental correction factor t induced power
    v_0 = 4.03     # Mean rotor induced velocity in hover
    delta = 0.012  # Profile drag coefficient

    # Constants that define power to hover
    P_0 = delta*p*s*A*np.power(omega , 3)*np.power(R , 3)/8
    P_i = (1 + k)*np.power(W , 1.5)/(np.sqrt(2*p*A))

    # Power consumption
    blade_profile = P_0*(1 + 3*np.power(v , 2)/np.power(Utip , 2))
    induced = P_i*np.sqrt((np.sqrt(1 + np.power(v , 4)/(4*np.power(v_0 , 4))) - np.power(v , 2)/(2*np.power(v_0 , 2))))
    parasite = 0.5*d_0*p*s*A*np.power(v,3)
    
    return blade_profile + induced + parasite

def energy(d, t, hover = True):
    """Returns the minimum energy consumed to travel a distance d in a time t. If hover == False, then the drone lands after arriving at the destination instead of hovering."""

    # if distance == 0 than hovers for t seconds
    if d==0:
        return Power(0)*t 
    
    v_min = d/t             # minimum speed to travel the distance d in time t
    v_aux = v_min
    E_aux = Power(v_aux)*t

    # numerical search for minimum power consumption between v_min and 30 m/s
    if hover:
        for v in np.arange(v_min, 30.0, 0.1): 
            if Power(v)*d/v + (t - d/v)*Power(0) < E_aux:
                v_aux = v
                E_aux = Power(v)*d/v + (t - d/v)*Power(0)
    else:
        for v in np.arange(v_min, 30.0, 0.1): 
            if Power(v)*d/v < E_aux:
                v_aux = v
                E_aux = Power(v)*d/v
    
    return E_aux