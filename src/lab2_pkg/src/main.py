#!/usr/bin/env python
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import math
import sys
import pdb

import rospy
import tf
import time
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tfs
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from autolab_core import RigidTransform, Point, NormalCloud, PointCloud
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
# import cvxpy as cvx
import Queue
from grasp_metrics import compute_force_closure, compute_gravity_resistance, compute_custom_metric

"""
Starter Commands
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800
roslaunch lab2_pkg baxter_left_hand_track.launch
rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true
    Only if you need MoveIt
rosrun lab2_pkg main.py
"""

def lookup_tag(tag_number):
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)
    while not rospy.is_shutdown():
        try:
            # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
            #     print 'Frames not found'
            #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
            #     exit(0)
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
            break
        except:
            continue
    return tag_pos[0:3] # only return x, y, z

def close_gripper():
    """closes the gripper"""
    right_gripper.close(block=True)
    rospy.sleep(1.0)

def open_gripper():
    """opens the gripper"""
    right_gripper.open(block=True)
    rospy.sleep(1.0)

def go_to_pose(pose):
    """Uses Moveit to go to the pose specified
    Parameters
    ----------
    pose : :obj:`geometry_msgs.msg.Pose`
        The pose to move to
    """
    right_arm.set_start_state_to_current_state()
    right_arm.set_pose_target(pose)
    right_arm.plan()
    right_arm.go()

def execute_grasp(T_object_gripper):
    """takes in the desired hand position relative to the object, finds the desired hand position in world coordinates.  
       Then moves the gripper from its starting orientation to some distance behind the object, then move to the 
       hand pose in world coordinates, closes the gripper, then moves up.  
    
    Parameters
    ----------
    T_object_gripper : :obj:`autolab_core.RigidTransform`
        desired position of gripper relative to the objects coordinate frame
    """
    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return
    # moveit go_to_pose()
    # change the planner used with set_planner_id() in main
    # YOUR CODE HERE

def contacts_to_baxter_hand_pose(contact1, contact2, approach_direction):
    """ takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
    
    Parameters
    ----------
    contact1 : :obj:`numpy.ndarray`
        position of finger1 in object frame
    contact2 : :obj:`numpy.ndarray`
        position of finger2 in object frame
    approach_direction : :obj:`numpy.ndarray`
        there are multiple grasps that go through contact1 and contact2.  This describes which 
        orientation the hand should be in

    Returns
    -------
    :obj:`autolab_core:RigidTransform` Hand pose in the object frame
    """
    # YOUR CODE HERE
    T_obj_gripper = None
    return T_obj_gripper

def sorted_contacts(vertices, normals, T_ar_object):
    """ takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality
    
    Parameters
    ----------
    vertices : :obj:`numpy.ndarray`
        nx3 mesh vertices
    normals : :obj:`numpy.ndarray`
        nx3 mesh normals
    T_ar_object : :obj:`autolab_core.RigidTransform`
        transform from the AR tag on the paper to the object

    Returns
    -------
    :obj:`list` of :obj:`numpy.ndarray`
        grasp_indices[i][0] and grasp_indices[i][1] are the indices of a pair of vertices.  These are randomly 
        sampled and their quality is saved in best_metric_indices
    :obj:`list` of int
        best_metric_indices is the indices of grasp_indices in order of grasp quality
    """
    N = 1024
    # prune vertices that are too close to the table so you dont smack into the table
    # you may want to change this line, to be how you see fit
    possible_indices = np.r_[:len(vertices)][vertices[:,2] + T_ar_object[2,3] >= 0.03]

    # Finding grasp via vertex sampling.  make sure to not consider grasps where the 
    # vertices are too big for the gripper

    # possible metrics: compute_force_closure, compute_gravity_resistance, compute_custom_metric
    metric = compute_custom_metric 
    grasp_indices = list()
    metric_scores = list()
    for i in range(N):
        candidate_indices = np.random.choice(possible_indices, 2, replace=False)
        point_dist = utils.length(vertices[candidate_indices[0]] - vertices[candidate_indices[1]])
        grasp_indices.append(candidate_indices)
        # assign lowest possible scores to impossible grasps
        if point_dist < MIN_HAND_DISTANCE or point_dist > MAX_HAND_DISTANCE:
            metric_scores.append(-sys.maxint - 1)
        else:
            contacts = vertices[candidate_indices]
            contact_normals = normals[candidate_indices]
            metric_scores.append(metric(contacts, contact_normals, NUM_FACETS, CONTACT_MU, CONTACT_GAMMA, OBJECT_MASS))

    # sort metrics and return the sorted order
    best_metric_indices = sorted(list(range(N)), key=lambda i: metric_scores[i], reverse=True)
    return grasp_indices, best_metric_indices


# probably don't need to change these (but confirm that they're correct)
MAX_HAND_DISTANCE = .04
MIN_HAND_DISTANCE = .01
CONTACT_MU = 0.5 # coefficient of friction
CONTACT_GAMMA = 0.1 # coefficient of tortion friction

# will need to change these
OBJECT_MASS = 0.25 # kg
# approximate the friction cone as the linear combination of `NUM_FACETS` vectors
NUM_FACETS = 32
# set this to false while debugging your grasp analysis
BAXTER_CONNECTED = False
# how many to execute
NUM_GRASPS = 5
OBJECT = "pawn"

# objects are different this year so you'll have to change this
# also you can use nodes/object_pose_publisher.py instead of finding the ar tag and then computing T_ar_object in this script.
if OBJECT == "pawn":
    MESH_FILENAME = '/home/cc/ee106b/sp18/class/ee106b-aax/ros_workspaces/lab2_ws/src/lab2_pkg/objects/pawn.obj'
    # ar tag on the paper
    TAG = 8
    # transform between the object and the AR tag on the paper
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.106])
    # how many times to subdivide the mesh
    SUBDIVIDE_STEPS = 0
elif OBJECT == 'nozzle':
    MESH_FILENAME = '/home/cc/ee106b/sp18/class/ee106b-aax/ros_workspaces/lab2_ws/src/lab2_pkg/objects/nozzle.obj'
    TAG = 9
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.035])
    SUBDIVIDE_STEPS = 1
elif OBJECT == "gearbox":
    MESH_FILENAME = '/home/cc/ee106b/sp18/class/ee106b-aax/ros_workspaces/lab2_ws/src/lab2_pkg/objects/gearbox.obj'
    TAG = 10
    T_ar_object = tfs.translation_matrix([-.09, -.065, 0.038])
    SUBDIVIDE_STEPS = 0

# Change to left if necessary!
if BAXTER_CONNECTED:
    right_gripper = baxter_gripper.Gripper('right')

listener = tf.TransformListener()
from_frame = 'base'
time.sleep(1)

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

    vertices = mesh.vertices # points on the obj
    triangles = mesh.triangles # combinations of vertex indices
    normals = mesh.normals # unit vectors normal to the object surface at their respective vertex

    ar_tag = lookup_tag(TAG)

    # find the transformation from the object coordinates to world coordinates... somehow

    # sample N points

    pdb.set_trace()

    # ??? = sorted_contacts(???)

    # # YOUR CODE HERE
    # for current_metric in ?????:
    #     # YOUR CODE HERE
        
    #     # visualize the mesh and contacts
    #     vis.figure()
    #     vis.mesh(mesh)
    #     vis.normals(NormalCloud(np.hstack((normal1.reshape(-1, 1), normal2.reshape(-1, 1))), frame='test'),
    #         PointCloud(np.hstack((contact1.reshape(-1, 1), contact2.reshape(-1, 1))), frame='test'))
    #     # vis.pose(T_obj_gripper, alpha=0.05)
    #     vis.show()
    #     if BAXTER_CONNECTED:
    #         repeat = True
    #         while repeat:
    #             execute_grasp(T_obj_gripper)
    #             repeat = bool(raw_input("repeat?"))

    # 500, 1200
    exit()