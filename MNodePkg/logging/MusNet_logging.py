"""
    Provide a set of methods to write Muscle Networks out to a file.
"""

def write_network(filename, network):
    """ Write a muscle network out to the given file. 

    Args:
        filename: file to write to
        network: muscle network to write out
    """
    with open(filename, "w") as f:
        f.write(str(network))

def write_networks(filename, networks):
    """ Write a list of muscle networks out to a given file.

    Args:
        filename: file to write to
        networks: muscle networks to write out
    """
    with open(filename, "w") as f:
        for network in networks:
            f.write(str(network))
            f.write("\n")
