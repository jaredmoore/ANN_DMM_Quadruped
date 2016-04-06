"""
    Class that reads in a muscle network file and returns a pandas data frame object.
"""

import math
import pandas
import StringIO
import sys
import xml.etree.ElementTree as etree

from MNodePkg.xmlparser import MusXMLParser

class DataImporter(object):
    """ Read in muscle network information and return it in the requested format. """

    @staticmethod
    def read_mus_nets_to_pandas_data_frame(input_file):
        """ Read in the muscle networks contained in the input file and return a pandas data frame.

        Args:
            input_file: file to read from
        Returns:
            pandas data frame containing information about the muscle network.
        """

        muscle_networks_list = []

        mn_strs = []
        mus_str = ""
        reading_mus = False
        fitnesses = []
        with open(input_file,'r') as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    # Parse the generation and fitness
                    fitnesses.append(float(pline.split(',')[1]))
                elif pline == "<MuscleNetwork>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleNetwork>":
                    mus_str += line
                    reading_mus = False
                    mn_strs.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line

        # Add the muscle networks to the data frame.
        for i,mus_net_str in enumerate(mn_strs):
            mn = DataImporter.mus_net_to_dictionary(mus_net_str)
            for mg in mn:
                for mnode in mg:
                    mnode['mus_net_num'] = i
                    mnode['fitness'] = fitnesses[i]
                    muscle_networks_list.append(mnode)
            if (i % 100 == 0):
                print("On Generation "+str(i))

        muscle_networks = pandas.DataFrame(muscle_networks_list)

        return muscle_networks

    @staticmethod
    def mus_net_to_dictionary(mus_net_str):
        """ Convert a muscle network string into a dictionary defining a muscle network.

        Args:
            mus_net_str: string defining a muscle network
        Returns:
            a 2d list containing definitions for the muscle nodes in a network

            format is the following:
            [[{mus_group_num,mus_node_num,x,y,type,a,b,c}]]
            dictionary defining a muscle network.
        """

        mn_obj = StringIO.StringIO(mus_net_str)

        mn_dicts = []

        tree = etree.parse(mn_obj)
        for i,mg_elem in enumerate(tree.findall('MuscleGroup')):
            new_mg = []
            for j,elem in enumerate(mg_elem.findall('MNode')):
                node = {'mus_node_num': j, 'mus_group_num': i}
                node['deg'] = float(elem.get('deg'))
                node['x'] = float("{0:.4f}".format(math.cos(math.radians(float(elem.get('deg')))))) 
                node['y'] = float("{0:.4f}".format(math.sin(math.radians(float(elem.get('deg')))))) 
                for gelem in elem.findall('GaussianNode'):
                    node['type'] = str("GaussianNode")
                    node['a'] = float(gelem.get('a'))
                    node['b'] = float(gelem.get('b'))
                    node['c'] = float(gelem.get('c'))
                new_mg.append(node)
            mn_dicts.append(new_mg)

        return mn_dicts
        




