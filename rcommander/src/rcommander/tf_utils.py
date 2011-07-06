import numpy as np
import tf.transformations as tr

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 
