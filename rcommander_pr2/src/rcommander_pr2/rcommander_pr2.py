import roslib; roslib.load_manifest('rcommander_pr2')
import rcommander as rc
import pr2_utils as pu
import tf 

tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rc.run(pr2, tf)
