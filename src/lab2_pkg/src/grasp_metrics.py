# may need more imports
import numpy as np
from utils import vec, adj

def compute_force_closure(contacts, normals, num_facets, mu, gamma, object_mass):
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
    # YOUR CODE HERE
    pass

# defined in the book on page 219
def get_grasp_map(contacts, normals, num_facets, mu, gamma):
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
    # YOUR CODE HERE
    pass

def contact_forces_exist(contacts, normals, num_facets, mu, gamma, desired_wrench):
    """ Compute whether the given grasp (at contacts with surface normals) can produce the desired_wrench.
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
    # YOUR CODE HERE
    pass

def compute_gravity_resistance(contacts, normals, num_facets, mu, gamma, object_mass):
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
    pass

def compute_custom_metric(contacts, normals, num_facets, mu, gamma, object_mass):
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
    # YOUR CODE HERE :)
    pass