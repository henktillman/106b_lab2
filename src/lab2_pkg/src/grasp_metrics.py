# may need more imports
import numpy as np
from utils import vec, adj
import utils
import sys, pdb

def compute_force_closure(contacts, normals, mu, gamma, object_mass):
    """ Compute the force closure of some object at contacts, with normal vectors stored in normals
        You can use the line method described in HW2.  if you do you will not need num_facets

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # calculate slope vector between points
    slope = utils.normalize(contacts[0] - contacts[1])

    score = 0

    # for each point
    for i in range(len(contacts)):
        # find angle of line with surface of object at the particular contact point
        # <x, y> = ||x||||y|| cos(th)
        line_angle = min(np.arccos(np.dot(normals[i], slope)), np.arccos(np.dot(normals[i], -slope)))

        # find the angle of the friction cone at the same point
        cone_angle = np.arctan(mu)

        # if the line angle is greater than the cone angle, then not force closure
        if cone_angle - line_angle < 0:
            score += -sys.maxint/10
        else:
            score += (cone_angle - line_angle)
    return score

# defined in the book on page 219
def get_grasp_map(contacts, normals, mu, gamma):
    """ Compute the grasp map given the contact points and their surface normals

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    :obj:`numpy.ndarray` grasp map
    """
    B = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]])
    grasp_map = None
    for i in range(len(contacts)):
        translation = contacts[i]

        orig_axis = np.array([0, 0, 1])
        final_axis = normals[i]

        v = np.cross(orig_axis, final_axis)
        ssc = np.matrix([[0, -v[2], v[1]],[v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation = np.eye(3) + ssc + np.dot(ssc, ssc)*(1 - np.dot(orig_axis, final_axis)) / np.linalg.norm(v)**2

        g = np.vstack((rotation, np.array([0, 0, 0])))
        c = np.vstack((np.matrix(translation).T, np.array([1])))
        g = np.hstack((g, c))
        adj = utils.adj(np.linalg.inv(g))

        if grasp_map is None:
            grasp_map = np.dot(adj, B)
        else:
            grasp_map = np.hstack((grasp_map, np.dot(adj, B)))
    return grasp_map

def contact_forces_exist(contacts, normals, mu, gamma, desired_wrench):
    """ Compute whether the given grasp (at contacts with surface normals) can produce the desired wrench.
        will be used for gravity resistance.

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    bool : whether contact forces can produce the desired_wrench on the object
    """
    def in_friction_cone(f):

        frictional = (f[0]**2 + f[1]**2)**0.5 <= mu * f[2]
        torsional = abs(f[3]) <= gamma * f[2]
        return frictional and torsional

    grasp_map = get_grasp_map(contacts, normals, mu, gamma)
    try:
        force_vecs = np.asarray(np.dot(np.linalg.pinv(grasp_map), desired_wrench))[0]
    except:
        return False
    f1 = force_vecs[0:4]
    f2 = force_vecs[4:]
    if in_friction_cone(f1) and in_friction_cone(f2):
        return True
    return False


def compute_gravity_resistance(contacts, normals, mu, gamma, object_mass):
    """ Gravity produces some wrench on your object.  Computes whether the grasp can produce and equal and opposite wrench

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE (contact forces exist may be useful here)
    gravity_wrench = np.array([0, 0, -9.81 * object_mass, 0, 0, 0])
    return contact_forces_exist(contacts, normals, mu, gamma, gravity_wrench)

def compute_custom_metric(contacts, normals, mu, gamma, object_mass):
    """ I suggest Ferrari Canny, but feel free to do anything other metric you find.

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    grasp_map = get_grasp_map(contacts, normals, mu, gamma)
    return np.linalg.det(np.dot(grasp_map, grasp_map.T))
