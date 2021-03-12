import pandas as pd
import numpy as np
params_dict = {'ID':['RW','N{}-Q{}'],'com_model':'soundv2','cap_com':0,
    'nei_sensing':15,'lit_sensing':3,
    'p_s2s':[0.001,0.01,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.99,0.999,1],
    'p_u2s':[0.001,0.01,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.99,0.999,1],
    'detectionDuration':[0,1],'FoV':2*np.pi,'rVel':10,'turn_prob':0.0025,
    'turn_prob_max':1,'turn_prob_min':0.0025,'abandon_prob':0.5,'picking_lit_dur':5,
    'capacity':5,'umin':0,'umax':1,'n_stddev':np.pi * 0.5,'n_mean':np.pi,'A0':299.1795,'Ae':48.1824,
    'alpha':0.1039,'noise_mean':[0,0.0583],'noise_std':[0,0.005],'queue_size':[1,40],'filter_type':'average_filter',
    'correction_mtd':'none','escape_dist':1,'call_scale_mult':[1,4],
    'call_scale_div':[1,100],'lit_threshold':5,'repel_scale_mult':[1,1],
    'repel_scale_div':[1,10],'log_rate':1,'max_step_size':0.025,'x':'x'}