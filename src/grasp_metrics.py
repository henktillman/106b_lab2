# may need imports

# can use the line method described in HW2
def compute_force_closure(contacts, normals, num_facets, mu, gamma, object_mass):
    # YOUR CODE HERE

# defined in the book on page 219
def get_grasp_map(contacts, normals, num_facets, mu, gamma):
    # YOUR CODE HERE

# will be used for gravity resistance. 
def contact_forces_exist(contacts, normals, num_facets, mu, gamma, desired_wrench):
    # YOUR CODE HERE

# gravity produces some wrench on your object.  can your grasp produce and equal and opposite wrench?
def compute_gravity_resistance(contacts, normals, num_facets, mu, gamma, object_mass):
    # YOUR CODE HERE (contact forces exist may be useful here)

# I suggest Ferrari Canny, but feel free to do anything other metric you find.  
def compute_custom_metric(contacts, normals, num_facets, mu, gamma, object_mass):
    # YOUR CODE HERE