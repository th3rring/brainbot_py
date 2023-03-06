import pybullet as pb
import numpy as np

from dataclasses import dataclass

@dataclass
class Transform:
    pos: np.array
    quaternion: np.array
    mat: np.matrix

    def __str__(self):
        return str(self.mat)

    def __mul__(self, other):
        rslt_p, rslt_q = pb.multiplyTransforms(self.pos, self.quaternion, other.pos, other.quaternion)

        return toTransform(rslt_p, rslt_q)

    def inv(self):
        rslt_p, rslt_q = pb.invertTransform(self.pos, self.quaternion)

        return toTransform(rslt_p, rslt_q)

    def rigidTF(self):
        return self.mat[0:3,:]
    
    def toList(self):
        return np.append(self.pos, self.quaternion).reshape(-1).tolist()


def toTransform(pos: np.array, orien: np.array) -> Transform:

    # Convert to numpy
    orien = np.array(orien)
    pos = np.array(pos)

    # Convert orientations to quaternion
    if len(orien) == 3 or orien.size == (3,1) or orien.size == (1,3) :
        orien = np.array(pb.getQuaternionFromEuler(orien))

    # Make position column vector
    pos_T = pos.reshape((3,1))

    # To rotation matrix
    rot_matrix = toRotationMatrix(orien)

    # Stable together matrix to form homogeneous transform
    t = np.append(rot_matrix,pos_T,1).reshape((3,4))
    t = np.matrix(np.append(t,[0,0,0,1]).reshape((4,4)))

    return Transform(pos, orien, t)

def transformMatrixToTransform(transform_mat: np.matrix) -> Transform:

    # Ensure transform matrix is corret size
    assert transform_mat.size != (3,4) and transform_mat.size != (4,4)

    # Extract components
    rot_matrix = np.matrix(transform_mat[0:3, 0:3])
    pos = transform_mat[0:3,3]

    # Convert matrix to quaternion
    # This is a bit retorical as I convert it back to a rotation matrix
    quaternion = rotationMatrixToQuaternion(rot_matrix)

    return toTransform(pos, quaternion)
    
def rotationMatrixToQuaternion(rot_mat: np.matrix) -> np.array:

    # There are many ways to do this
    # Taken from alternative method from this page https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    w = np.sqrt(max(0, 1 + rot_mat[0,0] + rot_mat[1,1] + rot_mat[2,2]))/2
    x = np.sqrt(max(0, 1 + rot_mat[0,0] - rot_mat[1,1] - rot_mat[2,2]))/2
    y = np.sqrt(max(0, 1 - rot_mat[0,0] + rot_mat[1,1] - rot_mat[2,2]))/2
    z = np.sqrt(max(0, 1 - rot_mat[0,0] - rot_mat[1,1] + rot_mat[2,2]))/2

    x *= np.sign(rot_mat[2,1] - rot_mat[1,2])
    y *= np.sign(rot_mat[0,2] - rot_mat[2,0])
    z *= np.sign(rot_mat[1,0] - rot_mat[0,1])

    return np.array([x,y,z,w])

def toRotationMatrix(orien:np.array) -> np.array:

    # Get quaternion orientation representation
    # Pybullet uses x,y,z
    # Pybyller uses x, y, z, w for quaternions
    if len(orien) == 3 or orien.size == (3,1) or orien.size == (1,3) :
        orien = np.array(pb.getQuaternionFromEuler(orien))

    rot_matrix = np.array(pb.getMatrixFromQuaternion(orien)).reshape((3,3))
                            
    return rot_matrix
