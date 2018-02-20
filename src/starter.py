#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import math
import sys

import rospy
import tf
import time
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tfs
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from core import RigidTransform, Point, NormalCloud, PointCloud
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from meshpy import ObjFile
warnings.filterwarnings("ignore", category=DeprecationWarning)
from visualization import Visualizer3D as vis
warnings.filterwarnings("ignore", category=DeprecationWarning)
from baxter_interface import gripper as baxter_gripper
from utils import vec, adj
import scipy
import copy
import sys
import cvxpy as cvx
import Queue

# probably don't need to change these (but confirm that they're correct)
MAX_HAND_DISTANCE = .04
MIN_HAND_DISTANCE = .01
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1

# will need to change these
OBJECT_MASS = ??? # kg
# approximate the friction cone as the linear combination of `NUM_FACETS` vectors
NUM_FACETS = 32
# set this to false while debugging your grasp analysis
BAXTER_CONNECTED = False
# how many to execute
NUM_GRASPS = 6
OBJECT = "spray"

# objects are different this year so you'll have to change this
if OBJECT == "spray":
    MESH_FILENAME = 'data/objects/spray.obj'
    # ar tag on the paper
    TAG = 8
    # transform between the object and the AR tag on the paper
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.106])
    # how many times to subdivide the mesh
    SUBDIVIDE_STEPS = 0
elif OBJECT == 'bar_clamp':
    MESH_FILENAME = 'data/objects/bar_clamp.obj'
    TAG = 9
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.035])
    SUBDIVIDE_STEPS = 1
elif OBJECT == "mount2":
    MESH_FILENAME = 'data/objects/mount2.obj'
    TAG = 10
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.038])
    SUBDIVIDE_STEPS = 0

if BAXTER_CONNECTED:
    right_gripper = baxter_gripper.Gripper('right')

listener = tf.TransformListener()
from_frame = 'base'
time.sleep(1)

def lookup_tag(tag_number):
    to_frame = 'ar_marker_{}'.format(tag_number)
    if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
        print 'Frames not found'
        print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
        exit(0)
    t = listener.getLatestCommonTime(from_frame, to_frame)
    tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
    return rigid_transform(tag_pos, tag_rot)

def close_gripper():
    right_gripper.close(block=True)
    rospy.sleep(1.0)

def open_gripper():
    right_gripper.open(block=True)
    rospy.sleep(1.0)

def go_to_pose(pose):
    right_arm.set_start_state_to_current_state()
    right_arm.set_pose_target(pose)
    right_arm.plan()
    right_arm.go()

# takes in the final position of the hand.  should move the gripper from its starting orientation
# to some distance behind the object, then move to the T_object_gripper pose, close the gripper, then move up.  
def execute_grasp(T_object_gripper):
    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return
    # YOUR CODE HERE

def contacts_to_baxter_hand_pose(contact1, contact2, approach_direction):
    # YOUR CODE HERE
    # takes the contacts positions in 3D space and returns the correct baxter hand position

# takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality
def sorted_contacts(???):
    # prune vertices that are too close to the table so you dont smack into the table
    possible_indices = np.r_[:len(vertices)][vertices[:,2] + T_ar_object[2,3] >= 0.03]

    # Finding grasp via vertex sampling.  make sure to not consider grasps where the 
    # vertices are too big for the gripper
    all_metrics = list()
    metric = compute_custom_metric
    grasp_indices = list()
    for i in range(?????):
        candidate_indices = np.random.choice(possible_indices, 2, replace=False)
        grasp_indices.append(candidate_indices)

        # YOUR CODE HERE
        all_metrics.append(????)

    return ???


if __name__ == '__main__':
    if BAXTER_CONNECTED:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_node')
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(5)
        
    # Main Code
    br = tf.TransformBroadcaster()

    # SETUP
    of = ObjFile(MESH_FILENAME)
    mesh = of.read()

    # We found this helped.  You may not.  I believe there was a problem with setting the surface normals.
    # I remember fixing that....but I didn't save that code, so you may have to redo it.  
    # You may need to fix that if you call this function.
    for i in range(SUBDIVIDE_STEPS):
        mesh = mesh.subdivide(min_tri_length=.02)

    vertices = mesh.vertices
    triangles = mesh.triangles
    normals = mesh.normals

    ??? = sorted_contacts(???)

    # YOUR CODE HERE
    for current_metric in ?????:
        # YOUR CODE HERE
        
        # visualize the mesh and contacts
        vis.figure()
        vis.mesh(mesh)
        vis.normals(NormalCloud(np.hstack((normal1.reshape(-1, 1), normal2.reshape(-1, 1))), frame='test'),
            PointCloud(np.hstack((contact1.reshape(-1, 1), contact2.reshape(-1, 1))), frame='test'))
        # vis.pose(T_obj_gripper, alpha=0.05)
        vis.show()
        if BAXTER_CONNECTED:
            repeat = True
            while repeat:
                execute_grasp(T_obj_gripper)
                repeat = bool(raw_input("repeat?"))

    # 500, 1200
    exit()
