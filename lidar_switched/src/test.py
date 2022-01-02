
import numpy as np
from scipy.spatial.transform import Rotation as R
def _generate_transform(odom_pos, odom_quat):
        tf = np.eye(4)
        tf[:3,:3] = odom_quat.as_matrix()
        tf[:3,3] = odom_pos
        return tf

left_to_ram = _generate_transform(np.asarray([1.549, 0.267, 0.543]), R.from_euler('zyx', np.asarray([2*np.pi/3, 0, 0])))
right_to_ram = _generate_transform(np.asarray([1.549, -0.267, 0.543]), R.from_euler('zyx',np.asarray([-2*np.pi/3, 0, 0])))
front_to_ram = _generate_transform(np.asarray([2.242, 0, 0.448]), R.from_euler('zyx', np.asarray([0, 0, 0])))

ram_to_cog = _generate_transform(np.asarray([-1.3206, 0.030188, -0.23598]), R.from_euler('zyx', np.asarray([0, 0, 0])))
cog_to_baselink = _generate_transform(np.asarray([0, 0, 0]), R.from_euler('zyx', np.asarray([0, 0, 0])))

left_global = np.eye(4) @ cog_to_baselink @ ram_to_cog  @ left_to_ram 
right_global = np.eye(4) @ cog_to_baselink @ ram_to_cog  @ right_to_ram 
front_global = np.eye(4) @ cog_to_baselink @ ram_to_cog  @ front_to_ram

print(left_global)
print(right_global)
print(front_global)
dtype = np.float32
itemsize = np.dtype(dtype).itemsize
print(itemsize)