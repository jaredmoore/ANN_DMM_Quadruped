"""
    Provides a set of methods to analyze a run.
"""

import argparse
import pandas
import sys

from os.path import expanduser

from MNodePkg.analysis.data_importer import DataImporter

from MNodePkg.xmlparser import MusXMLParser
from MNodePkg.visuals import Visuals
from MNodePkg.MNodes import MuscleGroup

class MusAnalysis(object):
    """ Provide a set of methods to analyze a muscle group run. """

    @staticmethod
    def analyze_best_individuals(filepath, indivs=-1, save_plots=0):
        """ Open and analyze the muscle groups from the provided file.

        Args:
            filepath: file to open and analyze
            indivs: indices of the individuals to look at (0 - gen 0, 10 - gen 10...)
        """
        best_ind_strs = []
        mus_str = ""
        reading_mus = False
        with open(filepath,"r") as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    pass
                elif pline == "<MuscleGroup>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleGroup>":
                    mus_str += line
                    reading_mus = False
                    best_ind_strs.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line

        mgs = []
        if indivs != -1:
            for i in indivs:
                mgs.append(MusXMLParser.parse_muscle_group(best_ind_strs[i]))
        else:
            mgs.append(MusXMLParser.parse_muscle_group(best_ind_strs[0]))
            mgs.append(MusXMLParser.parse_muscle_group(best_ind_strs[-1]))

        if save_plots:
            fpath = filepath[:filepath.rfind('/')+(filepath[filepath.rfind('/'):].find('_',1)-filepath[filepath.rfind('/'):].rfind('/'))]+'/'
            Visuals.plot_node_activations(mgs,filepath=fpath)
            Visuals.plot_aggregate_activations(mgs,filepath=fpath)
            Visuals.plot_node_positions(mgs,filepath=fpath)
        else:
            Visuals.plot_node_activations(mgs)
            Visuals.plot_aggregate_activations(mgs)
            Visuals.plot_node_positions(mgs)

    @staticmethod
    def analyze_best_individual_mn_placement(filepath, save_plots=0):
        """ Open and analyze the last muscle network placement from the provided file.

        Args:
            filepath: file to open and analyze
        """
        best_per_gen = []
        mus_str = ""
        reading_mus = False
        with open(filepath,"r") as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    pass
                elif pline == "<MuscleNetwork>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleNetwork>":
                    mus_str += line
                    reading_mus = False
                    best_per_gen.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line
        
        best_mn = MusXMLParser.parse_muscle_network(best_per_gen[-1])

        if save_plots:
            for i,mg in enumerate(best_mn.groups):
                fpath = filepath[:filepath.rfind('/')+(filepath[filepath.rfind('/'):].find('_',1)-filepath[filepath.rfind('/'):].rfind('/'))]+'/'
                Visuals.plot_node_positions(mg,filepath=fpath+str(i)+"_")
        else:
            for mg in best_mn.groups:
                Visuals.plot_node_activations(mg)
                Visuals.plot_aggregate_activations(mg)
                Visuals.plot_node_positions(mg)
                
    @staticmethod
    def analyze_best_individual_mn_aggregate_activations(filepath, save_plots=0):
        """ Open and analyze the last muscle network placement from the provided file.

        Args:
            filepath: file to open and analyze
        """
        best_per_gen = []
        mus_str = ""
        reading_mus = False
        with open(filepath,"r") as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    pass
                elif pline == "<MuscleNetwork>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleNetwork>":
                    mus_str += line
                    reading_mus = False
                    best_per_gen.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line
        
        best_mn = MusXMLParser.parse_muscle_network(best_per_gen[-1])

        if save_plots:
            for i,mg in enumerate(best_mn.groups):
                fpath = filepath[:filepath.rfind('/')+(filepath[filepath.rfind('/'):].find('_',1)-filepath[filepath.rfind('/'):].rfind('/'))]+'/'
                Visuals.plot_aggregate_activations(mg,filepath=fpath+str(i)+"_")
        else:
            for mg in best_mn.groups:
                Visuals.plot_node_activations(mg)
                Visuals.plot_aggregate_activations(mg)
                Visuals.plot_node_positions(mg)

    @staticmethod
    def analyze_best_individual_mn_mg_paths(filepath, save_plots=0):
        """ Open and analyze the last muscle network placement from the provided file.

        Args:
            filepath: file to open and analyze
        """
        best_per_gen = []
        mus_str = ""
        reading_mus = False
        with open(filepath,"r") as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    pass
                elif pline == "<MuscleNetwork>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleNetwork>":
                    mus_str += line
                    reading_mus = False
                    best_per_gen.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line
        
        best_mn = MusXMLParser.parse_muscle_network(best_per_gen[-1])

        if save_plots:
            for i,mg in enumerate(best_mn.groups):
                fpath = filepath[:filepath.rfind('/')+(filepath[filepath.rfind('/'):].find('_',1)-filepath[filepath.rfind('/'):].rfind('/'))]+'/'
                Visuals.plot_activation_path(mg,filepath=fpath+str(i)+"_")
        else:
            for mg in best_mn.groups:
                Visuals.plot_activation_path(mg)

    @staticmethod
    def analyze_best_individual_mus_node_placement(filepath,save_data=False,save_path="~/Desktop/"):
        """ Open and analyze the muscle networks from the provided file clustering the muscle nodes based on position.

        Args:
            filepath: file to open and analyze
        """
        mus_nets_df = DataImporter.read_mus_nets_to_pandas_data_frame(filepath)

        # Write out the data if requested:
        if save_data:
            mus_nets_df.to_csv(expanduser(save_path)+"/mus_node_filtered_data.dat", index=False, columns=["mus_net_num","mus_group_num","mus_node_num","fitness","x","y","type","a","b","c"])

        # Cluster the muscle nodes using minimum paired distance metric.
        # TODO

    @staticmethod
    def analyze_best_individual_mus_node_aggregate_activations(filepath,save_data=False,save_path="~/Desktop/"):
        """ Open the provided file and get the muscle group aggregate activations for each muscle network.

        Args:
            filepath: file to open and analyze
        """

        # Read in the muscle networks
        best_per_gen = []
        mus_str = ""
        reading_mus = False
        with open(filepath,"r") as f:
            for line in f:
                pline = line.strip()
                if len(pline) <= 0:
                    pass
                elif pline[0] != "<":
                    pass
                elif pline == "<MuscleNetwork>":
                    mus_str += line
                    reading_mus = True
                elif pline == "</MuscleNetwork>":
                    mus_str += line
                    reading_mus = False
                    best_per_gen.append(mus_str)
                    mus_str = ""
                elif reading_mus:
                    mus_str += line
        
        activations_list = []

        # Go through each muscle network and get the aggregate activations in x and y coordinates.
        x = [(-100+4*x)/100. for x in xrange(51)]
        for i,mn in enumerate(best_per_gen):
            if i % 100 == 0:
                print("On Generation "+str(i))
            mus_net = MusXMLParser.parse_muscle_network(mn)

            # Get the activations for the muscle network for each input value.
            # Format: tuples of x,y per muscle group per step
            step_values = [mus_net.get_avg_activations(x_val) for x_val in x]

            for x_val,step in zip(x,step_values):
                for j,group in enumerate(step):
                    activations_list.append({'mus_net_number':i,'mus_group_number':j,'inp_val':x_val,'act_x':group[0],'act_y':group[1]})

        activations_df = pandas.DataFrame(activations_list)

        if save_data:
            print("Starting Save")
            activations_df.to_csv(expanduser(save_path)+"/mus_net_aggregate_activation_filtered_data.dat", index=False, columns=['mus_net_number','mus_group_number','inp_val','act_x','act_y'])



