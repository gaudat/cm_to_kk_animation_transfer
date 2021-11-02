import bpy
import json
import re
import os

from mathutils import *
from math import *

# Dirty stuff

op_history = (None, None)

# Configs

scale_cm_to_kk = 0.2

# Functions for serialization to JSON

def matrix_to_python_list(mat):
    return [
        [mat[x][y] for y in range(3)]
        for x in range(3)
    ]
    
def vector_to_python_list(mat):
    return list(mat)

# Utility functions

def get_basis_matrix(object):
    X = object.x_axis
    Y = object.y_axis
    Z = object.z_axis
    return Matrix((
        (X.x, Y.x, Z.x),
        (X.y, Y.y, Z.y),
        (X.z, Y.z, Z.z)
        ))
        
def deformed(C, ob):
    dg = C.evaluated_depsgraph_get()
    ob = ob.evaluated_get(dg)
    return ob

def deformed_bone(C, arm_name, bone_name):
    return deformed(C, C.scene.objects[arm_name]).pose.bones[bone_name]

def bone(C, arm_name, bone_name):
    return C.scene.objects[arm_name].pose.bones[bone_name]

def bone_anim_vec_attr(self, C, arm_name, bone_name, vec_attr):
    ret = deformed_bone(C, arm_name, bone_name)
    ret = getattr(ret, vec_attr)
    if arm_name == self.cm_arm:
        return ret * scale_cm_to_kk
    if arm_name == self.kk_arm:
        # Permute axes
        return Vector((ret[0], -ret[2], ret[1])) 

def bone_anim_head(self, C, arm_name, bone_name):
    return bone_anim_vec_attr(self, C, arm_name, bone_name, 'head')
    
def bone_anim_tail(self, C, arm_name, bone_name):
    return bone_anim_vec_attr(self, C, arm_name, bone_name, 'tail')

def bone_anim_basis(self, C, arm_name, bone_name):
    # Get basis vectors of the bone in global frame
    ret = get_basis_matrix(deformed_bone(C, arm_name, bone_name))
    if arm_name == self.kk_arm:
        ret = Matrix(((ret[0][0], ret[0][1], ret[0][2]),
        (-ret[2][0], -ret[2][1], -ret[2][2]),
        (ret[1][0], ret[1][1], ret[1][2])))
    return ret

def bone_transform_basis(self, C, arm_name, bone_name):
    # == bone_anim_basis when no rotation is applied on object
    return bone_anim_basis(self, C, arm_name, bone_name) @ deformed_bone(C, arm_name, bone_name).rotation_quaternion.inverted().to_matrix()

def bone_set_loc(self, C, arm_name, bone_name, global_movement_vec):
    # Location basis is self but with zero rotation
    btb = bone_transform_basis(self, C, arm_name, bone_name)
    C.scene.objects[arm_name].pose.bones[bone_name].location += btb.inverted() @ global_movement_vec 
    C.scene.objects[arm_name].pose.bones[bone_name].keyframe_insert('location')
    
def bone_set_rot(self, C, arm_name, bone_name, global_rotate_mat):
    # Location basis is self but with zero rotation
    btb = bone_transform_basis(self, C, arm_name, bone_name)
    rot_local = btb.inverted() @ global_rotate_mat @ btb
    rot_local = rot_local.to_quaternion()
    C.scene.objects[arm_name].pose.bones[bone_name].rotation_quaternion = rot_local
    C.scene.objects[arm_name].pose.bones[bone_name].keyframe_insert('rotation_quaternion')
    
class TransferPoseCommon(bpy.types.Operator):
    cm_arm: bpy.props.StringProperty()
    kk_arm: bpy.props.StringProperty()
    json_fn: bpy.props.StringProperty()

    def load_tpose_basis(self,context, event):
        # Load T-Pose basis for armatures from external files
        with open(bpy.path.abspath('//') + self.json_fn) as jf:
            self.tpose_basis = json.loads(jf.read())
            
    def check_transform_basis(self, C, event):
        arm_name = self.kk_arm
        bone_name = C.selected_pose_bones[0].name
        self.report({'INFO'}, f'Bone basis: {bone_anim_basis(self, C, arm_name, bone_name)}')
        btb = bone_transform_basis(self, C, arm_name, bone_name)
        self.report({'INFO'}, f'BTB: {btb}')
            
    def transfer_location(self, cm_bone_name, kk_bone_name, lerp_amount=0):  
        context = self.context      
        cm_bone_loc = bone_anim_head(self, context, self.cm_arm, cm_bone_name).lerp(bone_anim_tail(self, context, self.cm_arm, cm_bone_name), lerp_amount)
        self.report({'DEBUG'}, f'CM bone location: {cm_bone_loc}')
        kk_bone_loc = bone_anim_head(self, context, self.kk_arm, kk_bone_name)
        self.report({'DEBUG'}, f'KK bone location: {kk_bone_loc}')
        btb = bone_transform_basis(self, context, self.kk_arm, kk_bone_name)
        self.report({'DEBUG'}, f'BTB: {btb}')
        # location update does not change btb
        # movement in btb = btb^-1 @ movement in global
        # movement in global = btb @ movement in btb
        bone_set_loc(self, context, self.kk_arm, kk_bone_name, cm_bone_loc - kk_bone_loc)
        
    def transfer_rotation(self, cm_bone_name, kk_bone_name, add_local_rotation=None):
        context = self.context
        t_pose_rot_global = Matrix(self.tpose_basis[self.cm_arm][cm_bone_name]['basis'])
        self.report({'DEBUG'}, f'CM T-pose rotation: {t_pose_rot_global}')
        cur_rot_global = bone_anim_basis(self, context, self.cm_arm, cm_bone_name)
        self.report({'DEBUG'}, f'CM pose rotation: {cur_rot_global}')
        kk_t_pose_rot_global = Matrix(self.tpose_basis[self.kk_arm][kk_bone_name]['basis'])
        self.report({'DEBUG'}, f'KK T-pose rotation: {kk_t_pose_rot_global}')
        # [cur_rot_global] = [T] [t_pose_rot_global]
        # [T] = [cur_rot_global] [t_pose_rot_global]^-1
        # [kk_rot_global] = [T] [kk_t_pose_rot_global]
        # [kk_rot_global] = [cur_rot_global] [t_pose_rot_global]^-1 [kk_t_pose_rot_global]
        kk_rot_global = cur_rot_global @ t_pose_rot_global.inverted() @ kk_t_pose_rot_global
        
        self.report({'DEBUG'}, f'KK pose rotation: {kk_rot_global}')
        bone_set_rot(self, context, self.kk_arm, kk_bone_name, kk_rot_global)
        if add_local_rotation is None:
            return
        rq = deformed_bone(bpy.context, self.kk_arm, kk_bone_name).rotation_quaternion
        rq = rq @ add_local_rotation.to_quaternion()
        bone(context, self.kk_arm, kk_bone_name).rotation_quaternion = rq
        context.scene.objects[self.kk_arm].pose.bones[kk_bone_name].keyframe_insert('rotation_quaternion')
    
    def match_orientation(self, cm_bone_name, kk_bone_name, extra_roll=0):
        # Rotates kk_bone_name such that they have the same basis
        context = self.context
        cur_rot_global = bone_anim_basis(self, context, self.cm_arm, cm_bone_name)
        kk_btb = bone_transform_basis(self, context, self.kk_arm, kk_bone_name)
        rq = (kk_btb.inverted() @ cur_rot_global)
        rq = rq.to_euler('YXZ')
        rq.y += extra_roll
        rq = rq.to_quaternion()
        self.report({'DEBUG'}, f'rq: {rq}')
        context.scene.objects[self.kk_arm].pose.bones[kk_bone_name].rotation_quaternion = rq
        context.scene.objects[self.kk_arm].pose.bones[kk_bone_name].keyframe_insert('rotation_quaternion')
    
    def match_leg_fk_roll(self, cm_bone_name, kk_bone_name, override_roll=None):
        context = self.context
        # Match absolute orientation but use relative bone roll
        t_pose_rot_global = Matrix(self.tpose_basis[self.cm_arm][cm_bone_name]['basis'])
        self.report({'INFO'}, f'CM T-pose rotation: {t_pose_rot_global}')
        cur_rot_global = bone_anim_basis(self, context, self.cm_arm, cm_bone_name)
        self.report({'INFO'}, f'CM pose rotation: {cur_rot_global}')
        # B_C = B_T @ A
        # A = B_T^-1 @ B_C        
        kk_btb = bone_transform_basis(self, context, self.kk_arm, kk_bone_name)
        self.report({'INFO'}, f'KK BTB: {kk_btb}')
        kk_rot = (kk_btb.inverted() @ cur_rot_global).to_euler('YXZ')
        if override_roll is not None:
            kk_rot.y = override_roll
        self.report({'INFO'}, f'Roll amount check: {kk_btb @ kk_rot.to_matrix()}')
        #bone_set_rot(self, context, self.kk_arm, kk_bone_name, kk_rot.to_matrix())
        context.scene.objects[self.kk_arm].pose.bones[kk_bone_name].rotation_quaternion = kk_rot.to_quaternion()
        context.scene.objects[self.kk_arm].pose.bones[kk_bone_name].keyframe_insert('rotation_quaternion')
    
    def reset_kk_arm(self, context, event):
        for pb in context.scene.objects[self.kk_arm].pose.bones:
            pb.location = Vector((0,0,0))
            pb.rotation_quaternion = Quaternion()
            
    def modal(self, context, event):
        global op_history
        self.context = context
        if event.type == 'ESC':
            self.report({'WARNING'}, 'Cancelled')
            return {'CANCELLED'}
        if self.running_gen is not None:
            try:
                next(self.running_gen)
                return {'PASS_THROUGH'}
            except StopIteration:
                self.running_gen = None
                self.current_state += 1
        if self.current_state >= len(self.op_stack):
            op_history = (self.bl_idname, 'FINISHED')
            return {'FINISHED'}
        fun = self.op_stack[self.current_state]
        self.report({'DEBUG'}, f'Running: {fun.__name__}')
        ret = fun(context, event)
        if '__next__' in dir(ret):
            self.running_gen = ret
            return {'PASS_THROUGH'}
        self.current_state += 1
        return {'PASS_THROUGH'}
    
    def execute(self, context):
        global op_history
        op_history = (self.bl_idname, 'STARTED')
        self.context = context
        # Suppose an animation is loaded on cm_arm
        self.op_stack = [
            self.reset_kk_arm,
            self.load_tpose_basis,
#            self.check_transform_basis,
            self.transfer_torso,
            self.transfer_left_leg,
            self.transfer_right_leg,
            self.transfer_spine,
            self.transfer_shoulder,
            self.transfer_left_arm,
#            self.transfer_left_arm_fk,
            self.transfer_right_arm,
#            self.transfer_right_arm_fk,
            self.transfer_fingers,
        ]
        self.running_gen = None
        self.current_state = 0
        

class TransferPose(TransferPoseCommon):
    bl_idname = "script.transfer_pose"
    bl_label = "Transfer pose from CM to KK"
    
    def transfer_torso(self, context, event):
        cm_bone_name = 'Bip01 Pelvis'
        kk_bone_name = 'torso'
        self.transfer_location(cm_bone_name, kk_bone_name)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
        
    def transfer_left_leg(self, context, event):
        # Knee endpoint: 3DOF to Bip01 L Calf 0%
        cm_bone_name = 'Bip01 L Calf'
        kk_bone_name = 'thigh_ik_target.L'
        self.transfer_location(cm_bone_name, kk_bone_name)
        yield
        # IK endpoint: 6DOF to Bip01 L Foot 50%
        cm_bone_name = 'Bip01 L Foot'
        kk_bone_name = 'foot_ik.L'
        lerp_amount = 0.1
        self.transfer_location(cm_bone_name, kk_bone_name, lerp_amount)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
        
    def transfer_left_leg_fk(self, context, event):
        # Disabled as pelvis deformation is as janky as ik
        cm_bone_name = 'Bip01 L Thigh'
        kk_bone_name = 'thigh_fk.L'
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
        yield
        cm_bone_name = 'Bip01 L Calf'
        kk_bone_name = 'shin_fk.L'
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
        yield
        # IK endpoint: 6DOF to Bip01 L Foot 50%
        cm_bone_name = 'Bip01 L Foot'
        kk_bone_name = 'foot_fk.L'
        lerp_amount = 0.1
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
        
    def transfer_shoulder(self, context, event):
        cm_bone_name = 'Bip01 L Clavicle'
        kk_bone_name = 'shoulder.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/2)
        yield
        cm_bone_name = 'Bip01 R Clavicle'
        kk_bone_name = 'shoulder.R'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        
    def transfer_right_leg(self, context, event):
        cm_bone_name = 'Bip01 R Calf'
        kk_bone_name = 'thigh_ik_target.R'
        self.transfer_location(cm_bone_name, kk_bone_name)
        yield
        cm_bone_name = 'Bip01 R Foot'
        kk_bone_name = 'foot_ik.R'
        lerp_amount = 0.1
        self.transfer_location(cm_bone_name, kk_bone_name, lerp_amount)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
            
    def transfer_left_arm_fk(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.L')['IK_FK'] = 1
        cm_bone_name = 'Bip01 L UpperArm'
        kk_bone_name = 'upper_arm_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/4)
        yield
        cm_bone_name = 'Bip01 L Forearm'
        kk_bone_name = 'forearm_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/4)
        yield
        cm_bone_name = 'Bip01 L Hand'
        kk_bone_name = 'hand_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi)
                    
    def transfer_right_arm_fk(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.R')['IK_FK'] = 1
        cm_bone_name = 'Bip01 R UpperArm'
        kk_bone_name = 'upper_arm_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 3*pi/4)
        yield
        cm_bone_name = 'Bip01 R Forearm'
        kk_bone_name = 'forearm_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 3*pi/4)
        yield
        cm_bone_name = 'Bip01 R Hand'
        kk_bone_name = 'hand_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 0)
        
    def transfer_left_arm(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.L')['IK_FK'] = 0
        cm_bone_name = 'Bip01 L Forearm'
        kk_bone_name = 'upper_arm_ik_target.L'
        self.transfer_location(cm_bone_name, kk_bone_name)
        # Move to joint position
        yield
        vec_a = deformed_bone(context, self.cm_arm, 'Bip01 L UpperArm').y_axis
        vec_b = deformed_bone(context, self.cm_arm, 'Bip01 L Forearm').y_axis
        away_vec = (vec_a - vec_b).normalized()
        self.report({'DEBUG'}, f'{vec_a} {vec_b} {away_vec}')
        bone_set_loc(self, context, self.kk_arm, kk_bone_name, away_vec)
        # Tune IK position to point away
        yield
        cm_bone_name = 'Bip01 L Hand'
        kk_bone_name = 'hand_ik.L'
        self.transfer_location(cm_bone_name, kk_bone_name, 0.1)
        self.match_orientation(cm_bone_name, kk_bone_name, pi)
                            
    def transfer_right_arm(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.R')['IK_FK'] = 0
        cm_bone_name = 'Bip01 R Forearm'
        kk_bone_name = 'upper_arm_ik_target.R'
        self.transfer_location(cm_bone_name, kk_bone_name)
        # Move to joint position
        yield
        vec_a = deformed_bone(context, self.cm_arm, 'Bip01 R UpperArm').y_axis
        vec_b = deformed_bone(context, self.cm_arm, 'Bip01 R Forearm').y_axis
        away_vec = (vec_a - vec_b).normalized()
        self.report({'DEBUG'}, f'{vec_a} {vec_b} {away_vec}')
        bone_set_loc(self, context, self.kk_arm, kk_bone_name, away_vec)
        # Tune IK position to point away
        yield
        cm_bone_name = 'Bip01 R Hand'
        kk_bone_name = 'hand_ik.R'
        self.transfer_location(cm_bone_name, kk_bone_name, 0.1)
        self.match_orientation(cm_bone_name, kk_bone_name)
        
    def transfer_fingers(self, context, event):
        add_roll = pi / 2
        finger_list = ('thumb', 'f_index', 'f_middle', 'f_ring', 'f_pinky')
        kk_finger_name_template = 'f_{}.{01-03}.{LR}'    
        cm_finger_name = 'Bip01 {LR} Finger{0-4}{empty, 1, 2}'
        for joint in range(3):
            for side in 'LR':
                for finger in range(5):
                    kk_finger_name = f'{finger_list[finger]}.{joint+1:02d}.{side}'
                    cm_finger_name = f'Bip01 {side} Finger{finger}'
                    if joint > 0:
                        cm_finger_name += f'{joint}'
                    self.report({'DEBUG'}, f'{cm_finger_name} {kk_finger_name}')
                    assert cm_finger_name in self.tpose_basis[self.cm_arm]
                    assert kk_finger_name in self.tpose_basis[self.kk_arm]
                    self.match_orientation(cm_finger_name, kk_finger_name, add_roll)
            yield
        self.match_orientation('Bip01 L Toe11', 'toe.L', add_roll)
        self.match_orientation('Bip01 R Toe11', 'toe.R', add_roll)
           
    def transfer_spine(self, context, event):
        cm_bone_name = 'Bip01 Spine0a'
        kk_bone_name = 'spine_fk.003'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'Bip01 Spine1'
        kk_bone_name = 'spine_fk.004'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'Bip01 Spine1a'
        kk_bone_name = 'spine_fk.005'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'Bip01 Neck'
        kk_bone_name = 'neck'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'Bip01 Head'
        kk_bone_name = 'head'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
    
    def invoke(self, context, event):
        self.json_fn = 'tpose_basis.json'
        self.cm_arm = 'body001.armature'
        self.kk_arm = 'combined.001'
        self.execute(context)
        
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
bpy.utils.register_class(TransferPose)



class TransferPoseMale(TransferPoseCommon):
    bl_idname = "script.transfer_pose_male"
    bl_label = "Transfer male pose from CM to KK"
    
    def transfer_torso(self, context, event):
        cm_bone_name = 'ManBip Pelvis'
        kk_bone_name = 'torso'
        self.transfer_location(cm_bone_name, kk_bone_name)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
        
    def transfer_left_leg(self, context, event):
        # Knee endpoint: 3DOF to Bip01 L Calf 0%
        cm_bone_name = 'ManBip L Calf'
        kk_bone_name = 'thigh_ik_target.L'
        self.transfer_location(cm_bone_name, kk_bone_name)
        yield
        # IK endpoint: 6DOF to Bip01 L Foot 50%
        cm_bone_name = 'ManBip L Foot'
        kk_bone_name = 'foot_ik.L'
        lerp_amount = 0.3
        self.transfer_location(cm_bone_name, kk_bone_name, lerp_amount)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
        
    def transfer_left_leg_fk(self, context, event):
        # Disabled as pelvis deformation is as janky as ik
        cm_bone_name = 'ManBip L Thigh'
        kk_bone_name = 'thigh_fk.L'
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
        yield
        cm_bone_name = 'ManBip L Calf'
        kk_bone_name = 'shin_fk.L'
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
        yield
        # IK endpoint: 6DOF to Bip01 L Foot 50%
        cm_bone_name = 'ManBip L Foot'
        kk_bone_name = 'foot_fk.L'
        lerp_amount = 0.1
        #self.match_leg_fk_roll(cm_bone_name, kk_bone_name)
            
    def transfer_left_arm_fk(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.L')['IK_FK'] = 1
        cm_bone_name = 'ManBip L UpperArm'
        kk_bone_name = 'upper_arm_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/4)
        yield
        cm_bone_name = 'ManBip L Forearm'
        kk_bone_name = 'forearm_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/4)
        yield
        cm_bone_name = 'ManBip L Hand'
        kk_bone_name = 'hand_fk.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi)
                    
    def transfer_right_arm_fk(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.R')['IK_FK'] = 1
        cm_bone_name = 'ManBip R UpperArm'
        kk_bone_name = 'upper_arm_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 3*pi/4)
        yield
        cm_bone_name = 'ManBip R Forearm'
        kk_bone_name = 'forearm_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 3*pi/4)
        yield
        cm_bone_name = 'ManBip R Hand'
        kk_bone_name = 'hand_fk.R'
        self.match_orientation(cm_bone_name, kk_bone_name, 0)
        
    def transfer_shoulder(self, context, event):
        cm_bone_name = 'ManBip L Clavicle'
        kk_bone_name = 'shoulder.L'
        self.match_orientation(cm_bone_name, kk_bone_name, pi/2)
        yield
        cm_bone_name = 'ManBip R Clavicle'
        kk_bone_name = 'shoulder.R'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
                
    def transfer_right_leg(self, context, event):
        cm_bone_name = 'ManBip R Calf'
        kk_bone_name = 'thigh_ik_target.R'
        self.transfer_location(cm_bone_name, kk_bone_name)
        yield
        cm_bone_name = 'ManBip R Foot'
        kk_bone_name = 'foot_ik.R'
        lerp_amount = 0.3
        self.transfer_location(cm_bone_name, kk_bone_name, lerp_amount)
        self.transfer_rotation(cm_bone_name, kk_bone_name)
            
    def transfer_left_arm(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.L')['IK_FK'] = 0
        cm_bone_name = 'ManBip L Forearm'
        kk_bone_name = 'upper_arm_ik_target.L'
        self.transfer_location(cm_bone_name, kk_bone_name)
        # Move to joint position
        yield
        vec_a = deformed_bone(context, self.cm_arm, 'ManBip L UpperArm').y_axis
        vec_b = deformed_bone(context, self.cm_arm, 'ManBip L Forearm').y_axis
        away_vec = (vec_a - vec_b).normalized()
        self.report({'DEBUG'}, f'{vec_a} {vec_b} {away_vec}')
        bone_set_loc(self, context, self.kk_arm, kk_bone_name, away_vec)
        # Tune IK position to point away
        yield
        cm_bone_name = 'ManBip L Hand'
        kk_bone_name = 'hand_ik.L'
        self.transfer_location(cm_bone_name, kk_bone_name, 0.3)
        self.match_orientation(cm_bone_name, kk_bone_name, pi)
                            
    def transfer_right_arm(self, context, event):
        bone(context, self.kk_arm, 'upper_arm_parent.R')['IK_FK'] = 0
        cm_bone_name = 'ManBip R Forearm'
        kk_bone_name = 'upper_arm_ik_target.R'
        self.transfer_location(cm_bone_name, kk_bone_name)
        # Move to joint position
        yield
        vec_a = deformed_bone(context, self.cm_arm, 'ManBip R UpperArm').y_axis
        vec_b = deformed_bone(context, self.cm_arm, 'ManBip R Forearm').y_axis
        away_vec = (vec_a - vec_b).normalized()
        self.report({'DEBUG'}, f'{vec_a} {vec_b} {away_vec}')
        bone_set_loc(self, context, self.kk_arm, kk_bone_name, away_vec)
        # Tune IK position to point away
        yield
        cm_bone_name = 'ManBip R Hand'
        kk_bone_name = 'hand_ik.R'
        self.transfer_location(cm_bone_name, kk_bone_name, 0.3)
        self.match_orientation(cm_bone_name, kk_bone_name)
        
    def transfer_fingers(self, context, event):
        add_roll = pi / 2
        finger_list = ('thumb', 'f_index', 'f_middle', 'f_ring', 'f_pinky')
        kk_finger_name_template = 'f_{}.{01-03}.{LR}'    
        cm_finger_name = 'ManBip {LR} Finger{0-4}{empty, 1, 2}'
        for joint in range(3):
            for side in 'LR':
                for finger in range(5):
                    kk_finger_name = f'{finger_list[finger]}.{joint+1:02d}.{side}'
                    cm_finger_name = f'ManBip {side} Finger{finger}'
                    if joint > 0:
                        cm_finger_name += f'{joint}'
                    self.report({'DEBUG'}, f'{cm_finger_name} {kk_finger_name}')
                    assert cm_finger_name in self.tpose_basis[self.cm_arm]
                    assert kk_finger_name in self.tpose_basis[self.kk_arm]
                    self.match_orientation(cm_finger_name, kk_finger_name, add_roll)
            yield
        self.match_orientation('ManBip L Toe0', 'toe.L', add_roll)
        self.match_orientation('ManBip R Toe0', 'toe.R', add_roll)
            
    def transfer_spine(self, context, event):
        cm_bone_name = 'ManBip Spine'
        kk_bone_name = 'spine_fk.003'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'ManBip Spine1'
        kk_bone_name = 'spine_fk.004'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'ManBip Spine2'
        kk_bone_name = 'spine_fk.005'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'ManBip Neck'
        kk_bone_name = 'neck'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
        yield
        cm_bone_name = 'ManBip Head'
        kk_bone_name = 'head'
        self.match_orientation(cm_bone_name, kk_bone_name, -pi/2)
    
    def invoke(self, context, event):
        self.json_fn = 'tpose_basis_male.json'
        self.cm_arm = 'mbody.armature.001'
        self.kk_arm = 'combined'
        self.execute(context)
        
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
bpy.utils.register_class(TransferPoseMale)

class SaveTPoseBasisCommon(bpy.types.Operator):
    cm_arm: bpy.props.StringProperty()
    kk_arm: bpy.props.StringProperty()
    json_fn: bpy.props.StringProperty()

    def execute(self, context):
        # Suppose both models are in the same pose (T-Pose)
        self.cm_arm = cm_arm
        self.kk_arm = kk_arm
        out_dict = {}
        for arm in [self.cm_arm, self.kk_arm]:
            out_dict[arm] = {}
            for b in deformed(context, context.scene.objects[arm]).pose.bones:
                out_dict[arm][b.name] = {}
                out_dict[arm][b.name]['basis'] = matrix_to_python_list(bone_anim_basis(self, context, arm, b.name))
                out_dict[arm][b.name]['head'] = vector_to_python_list(bone_anim_head(self, context, arm, b.name))
                out_dict[arm][b.name]['tail'] = vector_to_python_list(bone_anim_tail(self, context, arm, b.name))
        with open(bpy.path.abspath('//') + self.json_fn, "x") as jf:
            jf.write(json.dumps(out_dict))
        return {'FINISHED'}

class SaveTPoseBasis(SaveTPoseBasisCommon):
    bl_idname = "script.save_tpose_basis"
    bl_label = "Save T-pose basis"
    
    def invoke(self, context, event):
        self.cm_arm = 'body001.armature' # CM female
        self.kk_arm = 'combined.001' # KK female
        self.json_fn = 'tpose_basis.json'
        return self.execute(context)

bpy.utils.register_class(SaveTPoseBasis)

class SaveTPoseBasisMale(SaveTPoseBasisCommon):
    bl_idname = "script.save_tpose_basis_male"
    bl_label = "Save male T-pose basis"
    
    def invoke(self, context, event):
        self.cm_arm = 'mbody.armature.001'
        self.kk_arm = 'combined'
        self.json_fn = 'tpose_basis_male.json'
        return self.execute(context)

bpy.utils.register_class(SaveTPoseBasisMale)

class LoadAnimation(bpy.types.Operator):
    bl_idname = "script.load_animation"
    bl_label = "Load .anm file into armatures"
    
    cm_arm: bpy.props.StringProperty()
    cm_anm: bpy.props.StringProperty()
    
    def execute(self, context):
        # Select cm_arm
        context.view_layer.objects.active = context.scene.objects[self.cm_arm]
        # Load animation onto it
        bpy.ops.import_anim.import_cm3d2_anm(filepath=self.cm_anm)
    
        return {'FINISHED'}
        
bpy.utils.register_class(LoadAnimation)

class CreateAction(bpy.types.Operator):
    bl_idname = "script.create_action"
    bl_label = "Create new Action on an armature"
    
    arm: bpy.props.StringProperty()
    action_name: bpy.props.StringProperty()
    
    def execute(self, context):
        if self.action_name in bpy.data.actions:
            act = bpy.data.actions[self.action_name]
        else:
            act = bpy.data.actions.new(self.action_name)
        act.use_fake_user = True
        context.scene.objects[self.arm].animation_data.action = act
        return {'FINISHED'}
    
bpy.utils.register_class(CreateAction)
    
class TransferAnimation(bpy.types.Operator):
    bl_idname = 'script.transfer_animation'
    bl_label = 'Transfer animation'
                
    anm: bpy.props.StringProperty()
    
    def modal(self, context, event):
        global op_history
        self.context = context
        if context.scene.frame_current > context.scene.frame_end:
            op_history = (self.bl_idname, 'FINISHED')
            return {'FINISHED'}
        if event.type == 'ESC':
            self.report({'WARNING'}, 'Cancelled')
            return {'CANCELLED'}
        if self.next_subop_i >= len(self.subop_list):
            context.scene.frame_current += 1
            self.report({'INFO'}, f'Frame {context.scene.frame_current}')
            self.next_subop_i = 0
            return {'PASS_THROUGH'}
        if self.running_subop is None:
            # Start subop
            subop = self.subop_list[self.next_subop_i]
            self.report({'DEBUG'}, f'SUBOP {subop}')
            self.running_subop = subop
            ret = subop[0]('INVOKE_DEFAULT')
            assert ret == {'RUNNING_MODAL'}
            return {'PASS_THROUGH'}
        # Running subop until it ends
#        ret = self.running_subop.modal(context, event)
        if op_history[1] == 'FINISHED':
            self.report({'DEBUG'}, f'{self.running_subop} {op_history}')
            self.running_subop = None
            self.next_subop_i += 1
            return {'PASS_THROUGH'}
        return {'PASS_THROUGH'}

    def execute(self, context):
        self.context = context
        self.is_male = re.search('[^a-z]m[^a-z]', self.anm) is not None
        if self.is_male:
            self.cm_arm = 'mbody.armature.001'
            self.kk_arm = 'combined'
            self.transfer_pose = (bpy.ops.script.transfer_pose_male,
                {
                'json_fn': 'tpose_basis_male.json',
                'cm_arm': self.cm_arm,
                'kk_arm': self.kk_arm
            })
        else:
            self.cm_arm = 'body001.armature'
            self.kk_arm = 'combined.001'
            self.transfer_pose = (bpy.ops.script.transfer_pose,
                {
                'json_fn': 'tpose_basis.json',
                'cm_arm': self.cm_arm,
                'kk_arm': self.kk_arm
            })
        
        self.running_subop = None
        self.subop_list = [self.transfer_pose]
        self.next_subop_i = 0
        
        # Load animation
        bpy.ops.script.load_animation(
            cm_arm=self.cm_arm,
            cm_anm=self.anm
        )

        # Create new animation
        bpy.ops.script.create_action(
            arm=self.kk_arm,
            action_name=self.anm.replace('\\','/').split('/')[-1]
        )
        
        context.scene.frame_current = 0

    def invoke(self, context, event):
        self.execute(context)
        
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
bpy.utils.register_class(TransferAnimation)


def gen_by_ext(root_folder, extension, exclude=None):
    if exclude is None:
        exclude = set()
    for cd, sd, sf in os.walk(root_folder):
        for f in sf:
            if f.endswith('.'+extension):
                yield cd.replace('\\', '/') + '/' + f

class TransferAnimationsFromFolder(bpy.types.Operator):
    bl_idname = 'script.transfer_animation_from_folder'
    bl_label = 'Transfer animation from folder'
                
    folder: bpy.props.StringProperty()
    
    def modal(self, context, event):
        global op_history
        self.context = context
        if self.no_new_tasks():
            return {'FINISHED'}
        if event.type == 'ESC':
            self.report({'WARNING'}, 'Cancelled')
            return {'CANCELLED'}
        if self.running_task is None:
            self.start_new_task()
            return {'PASS_THROUGH'}
        else:
            return self.check_task()

    def no_new_tasks(self):
        if self.next_task is None:
            try:
                self.next_task = next(self.file_gen)
                return False
            except StopIteration:
                return True
        return False

    def start_new_task(self):
        if self.next_task is None:
            self.no_new_tasks()
        # self.next_task must have something here
        # Start task here
        assert self.running_task is None
        # Start task here
        self.report({'INFO'}, f'Anm file: {self.next_task}')
        try:
            action_name = self.next_task.replace('\\', '/').split('/')[-1]
            bpy.data.actions[action_name]
            self.running_task = None
            # If no exception thrown, this anm clip is already processed
            # Consume the task
            self.next_task = None
            return
        except KeyError:
            pass
        self.running_task = bpy.ops.script.transfer_animation(
            'INVOKE_DEFAULT',
            anm=self.next_task
            )
        # Consume self.next_task after it is used
        self.next_task = None
        assert self.running_task == {'RUNNING_MODAL'}

    def check_task(self):
        global op_history
        if op_history[0] == 'SCRIPT_OT_transfer_animation':
            if op_history[1] != 'FINISHED':
                return {op_history[1]}
            # Task is finished
            # Unset running task
            self.running_task = None
        # Continue existing task
        return {'PASS_THROUGH'}

    def execute(self, context):
        self.context = context
        self.file_gen = gen_by_ext(self.folder, 'anm')
        self.next_task = next(self.file_gen)
        self.running_task = None

    def invoke(self, context, event):
        self.execute(context)
        
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
bpy.utils.register_class(TransferAnimationsFromFolder)
bpy.ops.script.transfer_animation_from_folder('INVOKE_DEFAULT', folder=r'C:\Users\anon\Desktop\kkanim\anms\18700')

# [act.__setattr__('use_fake_user', True) for act in bpy.data.actions if act.name.endswith('.anm')]