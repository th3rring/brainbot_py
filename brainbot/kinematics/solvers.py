import numpy as np
import ur5_ikfast
import time

from brainbot.utils.tf import Transform, toTransform, transformMatrixToTransform
from brainbot.constants import HALF_PI, PI


class Ur5Kinematics:

    def __init__(self, ee_to_tool_tip_tf=toTransform([0, 0, 0], [0, 0, 0]),
                 world_to_base_tf=toTransform([0, 0, 0], [0, 0, 0]),
                 verbose=True):
        self.ur5_kin_ = ur5_ikfast.PyKinematics("ur5")
        self.n_dofs = self.ur5_kin_.getDOF()
        self.verbose = verbose

        self.world_to_base_tf = world_to_base_tf
        self.ee_to_tool_tip_tf = ee_to_tool_tip_tf

        self.ee_offset_ = toTransform([0, 0, 0], [0, -HALF_PI, HALF_PI])

    def forward(self, joint_config: np.array) -> Transform:

        # Ensure that we have correct no of joints
        assert len(joint_config) == self.n_dofs

        # Note that we do not have to worry about a bijection of joints
        # The kinematics match with the UR5 we use

        start = time.time()
        fk_val = self.ur5_kin_.forward(joint_config)
        end = time.time()

        if self.verbose:
            print(f"FK sol in {end-start}")

        # 3x4 rigid transformation matrix
        fk_val = np.matrix(fk_val).reshape(3, 4)

        # This performs am unnecessary conversion between
        base_to_ee = transformMatrixToTransform(fk_val)

        # Multiply tfs for full transform
        return self.world_to_base_tf * base_to_ee * self.ee_offset_ * self.ee_to_tool_tip_tf

    def inverse(self, ee_pose: Transform) -> np.array:

        # Get ik transform for query, this is the body to ee link transform with a rotation offset for the ee
        ik_t = self.world_to_base_tf.inv() * ee_pose * self.ee_to_tool_tip_tf.inv() * \
            self.ee_offset_.inv()

        # Massage into correct form
        inverse_input = ik_t.rigidTF().reshape(-1).tolist()[0]

        # Compute IK
        start = time.time()
        joint_configs = self.ur5_kin_.inverse(inverse_input)
        end = time.time()
        if self.verbose:
            print(f"IK sol in {end-start}")

        n_solutions = int(len(joint_configs)/self.n_dofs)
        joint_configs = np.asarray(joint_configs).reshape(
            n_solutions, self.n_dofs)

        return joint_configs
