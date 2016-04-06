"""
    XML parser to construct a muscle node object.
"""

import StringIO
import xml.etree.ElementTree as etree

import sys

from MNodePkg.MNodes import MuscleGroup
from MNodePkg.MNodes import MuscleNetwork
from MNodePkg.MNodes import IndirectMuscleNetwork
from MNodePkg.MNodes import IndirectMirroringMuscleNetwork

class MusXMLParser(object):
    """ Parse xml containing muscle definitions. """

    @staticmethod
    def parse_muscle_group(mg_str):
        """ Parse an xml string defining a muscle group.

        Args:
            mg_str: xml string representing a muscle group.

        Returns:
            muscle group object
        """
        output = StringIO.StringIO(mg_str)

        new_mg = MuscleGroup(num_nodes=0)

        tree = etree.parse(output)
        for elem in tree.findall('MNode'):
            node_list = {'deg':float(elem.get('deg'))}
            for gelem in elem:
                node_list['params'] = [float(gelem.get('a')), float(gelem.get('b')), float(gelem.get('c'))]
            new_mg.create_node(deg=node_list['deg'],params=node_list['params'])
        return new_mg

    @staticmethod
    def parse_muscle_network(mn_str):
        """ Parse an xml string defining a muscle network.

        Args:
            mn_str: xml string representing a muscle network.

        Returns:
            muscle network
        """
        output = StringIO.StringIO(mn_str)

        tree = etree.parse(output)

        # Figure out if reading a MuscleNetwork or IndirectMuscleNetwork
        new_mn = ""
        if tree.getroot().tag == "MuscleNetwork":
            new_mn = MuscleNetwork()
        elif tree.getroot().tag == "IndirectMuscleNetwork" or tree.getroot().tag == "IndirectMirroringMuscleNetwork":
            ind_map = []
            for indmap in tree.findall('IndMap'):
                ind_map = [int(i) for i in indmap.text.split(',')]

            if tree.getroot().tag == "IndirectMuscleNetwork":
                new_mn = IndirectMuscleNetwork(indirect_cons=ind_map)
            elif tree.getroot().tag == "IndirectMirroringMuscleNetwork":
                mir_vals = []
                for mirmap in tree.findall('MirMap'):
                    mir_vals = [int(i) for i in mirmap.text.split(',')]
                new_mn = IndirectMirroringMuscleNetwork(indirect_cons=ind_map,mirror_vals=mir_vals)

        for mg_elem in tree.findall('MuscleGroup'):
            new_mg = MuscleGroup(num_nodes=0)
            for elem in mg_elem.findall('MNode'):
                node_list = {'deg':float(elem.get('deg'))}
                for gelem in elem:
                    node_list['params'] = [float(gelem.get('a')), float(gelem.get('b')), float(gelem.get('c'))]
                new_mg.create_node(deg=node_list['deg'],params=node_list['params'])
            new_mn.add_group(new_mg)
        return new_mn
