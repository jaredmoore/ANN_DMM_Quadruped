"""
    Implements a network of muscle groups.
"""

import random

from m_group import MuscleGroup

class MuscleNetwork(object):
    """ A network containing muscle groups for controlling an individual.

    Attributes:
        groups: list of muscle group objects
    """

    def __init__(self,num_groups=0,num_nodes=0,gid=-1,crossover_type=0,params=[]):
        """ Initialize a muscle network.

        Args:
            num_groups: number of muscle groups to randomly create.
            num_nodes: list of ints defining the number of nodes for each group
            gid: genome id of associated meta-controller (NEAT implmentation)
        """
        if num_groups:
            if params:
                self.groups = [MuscleGroup(num_nodes=num_nodes[i],params=params[i]) for i in xrange(num_groups)]
            else:
                self.groups = [MuscleGroup(num_nodes=num_nodes[i]) for i in xrange(num_groups)]
        else:
            self.groups = []
        self.gid = gid
        self.crossover_type = 0 # 0 for random shuffle, 1 for one point crossover

    def __str__(self):
        ret = "<MuscleNetwork>\n"
        ret+= "<GID>"+str(self.gid)+"</GID>\n"
        for group in self.groups:
            ret += str(group)
        ret += "</MuscleNetwork>\n"
        return ret

    def copy(self):
        """ Copy the muscle network into a new muscle network. """
        mn = MuscleNetwork(gid=self.gid)
        for group in self.groups:
            mg = group.copy()
            mn.groups.append(mg)
        return mn

    def add_group(self,mg):
        """ Add a copy of a msucle group to the muscle network.

        Args:
            mg: muscle group to copy to the network
        """
        self.groups.append(mg.copy())

    def create_group(self,**kwargs):
        """ Add a newly created muscle group to the network.

        Args:
            **kwargs: keywords to build the network from.
        """
        self.groups.append(MuscleGroup(num_nodes=kwargs['num_nodes']))

    def mutate(self,mut_perc=0):
        """ Mutate the muscle network.

        If a mutation percentage is set, go through each muscle group and decide 
        whether or not to mutate it.  This is based on the total number of parameters
        in that muscle group (num_nodes*node_params+1(deg)) multiplied by the mutation
        percentage.  Large groups will likely always get mutated, but that should be
        allowable as they have a high number of parameters

        Otherwise, select one muscle group randomly to call mutate on.

        Args:
            mut_perc: mutation percentage per parameter.
        """
        if mut_perc:
            for group in self.groups:
                group.mutate(position=True,mut_perc=mut_perc)
        else:
            self.groups[random.randint(0,len(self.groups)-1)].mutate(position=True)

    def crossover(self,sec_network):
        """ Crossover two muscle networks.

        Crossover occurs at the muscle group level.  We therefore do not break the individual
        muscle groups up.

        Args:
            sec_network: network to perform crossover with.
        """
        # Ensure that the groups are the same size.
        if(len(self.groups) != len(sec_network.groups)):
            print("Networks not the same number of groups!")
            return self.copy()

        new_mn = MuscleNetwork(gid=self.gid)

        # Don't crossover if only one group.
        if len(self.groups) <= 1:
            if random.random() <= 0.5:
                return self.copy()
            else:
                return sec_network.copy()
        elif len(self.groups) == 2: # Handle only one position crossover.
            if random.random() <= 0.5:
                new_mn.add_group(self.groups[0].copy())
                new_mn.add_group(sec_network.groups[1].copy())
            else:
                new_mn.add_group(sec_network.groups[0].copy())
                new_mn.add_group(self.groups[1].copy())
            return new_mn.copy()

        # Handle crossover in any other case.
        if self.crossover_type == 0:
            new_mn = self._random_crossover(sec_network,new_mn)
        else: # Note: Change to 1 if ever implementing other types of crossover.
            new_mn = self._two_pt_crossover(sec_network,new_mn)

        return new_mn

    def _random_crossover(self,sec_network,new_mn):
        """ Crossover two networks using random selection of genetic elements.

        Args:
            sec_network: network to perform crossover with

        Returns:
            new muscle network comprised of genetic material from both parents
        """

        for i in xrange(len(self.groups)):
            if random.random() <= 0.5:
                new_mn.add_group(self.groups[i].copy())
            else:
                new_mn.add_group(sec_network.groups[i].copy())
        return new_mn

    def _two_pt_crossover(self,sec_network,new_mn):
        """ Crossover two networks using two point crossover.

        Args:
            sec_network: network to perform crossover with.
            new_mn: new child network

        Returns:
            new muscle network comprised of genetic material from both parents
        """

        xover_pos = sorted(random.sample([i for i in xrange(len(self.groups))],2))
        inner = random.random() # Whether we cross between or outside the positions.
        for i in xrange(len(self.groups)):
            if inner <= 0.5:
                if (i < xover_pos[0] or i >= xover_pos[1]):
                    new_mn.add_group(self.groups[i].copy())
                else:
                    new_mn.add_group(sec_network.groups[i].copy())
            else:
                if (i >= xover_pos[0] and i < xover_pos[1]):
                    new_mn.add_group(self.groups[i].copy())
                else:
                    new_mn.add_group(sec_network.groups[i].copy())
        return new_mn

    def get_raw_activations(self,inp,group_num=0):
        """ Get the raw activations for each muscle group.

        If group_num is specified then get only that node.  Otherwise, get the activations
        for all groups and return in a 2d list.

        Args:
            inp: input activation value
            group_num: node number to get activation of.

        Returns:
            list containing either one or all activations.
        """
        if group_num:
            return self.groups[group_num].get_raw_activations(inp)
        else:
            act_list = []
            for group in self.groups:
                act_list.append(group.get_raw_activations(inp))
 
    def get_activations(self,inp,group_num=0):
        """ Get the activations by axis for each muscle group.

        Args:
            group_num: group number to get the activations of
            inp: input activation value

        Returns:
            list containing one or all activations
        """
        if group_num:
            xacts, yacts = self.groups[group_num].get_activations(inp)
            return [xacts,yacts]
        else:
            activations = []
            for group in self.groups:
                xacts, yacts = group.get_activations(inp)
                activations.append([xacts,yacts])
            return activations

    def get_avg_activations(self,inp,group_num=0,inputs=0):
        """ Get the average activations for an entire muscle group.

        Args:
            group_num: only get this group if specified
            inp: input value

        Returns:
            list of activations
        """
        if inputs != 0:
            acts = []
            for inp, group in zip(inputs,self.groups):
                x,y = group.get_avg_activations(inp)
                acts.append([x,y])
            return acts
        else:
            if group_num:
                x,y = self.groups[group_num].get_avg_activations(inp)
                return [x,y]
            else:
                acts = []
                for group in self.groups:
                    x,y = group.get_avg_activations(inp)
                    acts.append([x,y])
                return acts

    def get_avg_activations_ind_mus_nodes(self,inp,group_num=0,inputs=0):
        """ Get the average activations for an entire muscle group with inputs derived for each muscle node.

        Args:
            group_num: only get this group if specified
            inp: input value
            inputs: lists of inputs to go to each muscle group

        Returns:
            list of activations
        """
        if inputs != 0:
            acts = []
            for inp, group in zip(inputs,self.groups):
                x,y = group.get_avg_activations_ind_mus_nodes(inp)
                acts.append([x,y])
            return acts
        else:
            if group_num:
                x,y = self.groups[group_num].get_avg_activations_ind_mus_nodes(inp)
                return [x,y]
            else:
                acts = []
                for group in self.groups:
                    x,y = group.get_avg_activations_ind_mus_nodes(inp)
                    acts.append([x,y])
                return acts

class IndirectMuscleNetwork(MuscleNetwork):
    """ Extension of the MuscleNetwork class that includes indirect mapping from genome to outputs. """

    def __init__(self,num_groups=0,num_nodes=0,gid=-1,crossover=0,num_indirect=0,indirect_cons=[]):
        """ Initialize a muscle network.

        Args:
            num_groups: number of muscle groups to randomly create.
            num_nodes: list of ints defining the number of nodes for each group
            gid: genome id of associated meta-controller (NEAT implmentation)
            crossover: type of crossover to use 0 - random 1 - two point
            num_indirect: number of indirect connections to connect to outputs
            indirect_cons: list to specify how the indirect connections should initially be made
        """
        super(IndirectMuscleNetwork, self).__init__(num_groups,num_nodes,gid,crossover)
        
        # Setup the indirect outputs.
        if indirect_cons:
            self.indirect_cons = indirect_cons
        else:
            self.indirect_cons = [i for i in xrange(num_groups)]

    def __str__(self):
        ret = "<IndirectMuscleNetwork>\n"
        ret+= "<GID>"+str(self.gid)+"</GID>\n"
        ret += "<IndMap>"+str(', '.join(str(item) for item in self.indirect_cons))+"</IndMap>\n"
        for group in self.groups:
            ret += str(group)
        ret += "</IndirectMuscleNetwork>\n"
        return ret

    def copy(self):
        """ Copy the indirect muscle network into a new muscle network. """
        mn = IndirectMuscleNetwork(gid=self.gid,indirect_cons=self.indirect_cons)
        for group in self.groups:
            mg = group.copy()
            mn.groups.append(mg)
        return mn

    def mutate(self,mut_perc=0.025):
        """ Mutate the muscle network.

        Args:
            mut_perc: what the percentage of mutation should be. 
        """
        super(IndirectMuscleNetwork, self).mutate(mut_perc=mut_perc)

        # Mutate the indirect connections.
        for index,con in enumerate(self.indirect_cons):
            if random.random() <= mut_perc/4.:
                self.indirect_cons[index] = random.randint(0,len(self.indirect_cons)-1)

    def crossover(self,sec_network):
        """ Crossover two muscle networks.

        Crossover occurs at the muscle group level.  We therefore do not break the individual
        muscle groups up.

        Args:
            sec_network: network to perform crossover with.
        """
        # Ensure that the groups are the same size.
        if(len(self.groups) != len(sec_network.groups)):
            print("Networks not the same number of groups!")
            return self.copy()

        new_mn = IndirectMuscleNetwork(gid=self.gid,indirect_cons=self.indirect_cons)

        # Don't crossover if only one group.
        if len(self.groups) <= 1:
            if random.random() <= 0.5:
                return self.copy()
            else:
                return sec_network.copy()
        elif len(self.groups) == 2: # Handle only one position crossover.
            if random.random() <= 0.5:
                new_mn.add_group(self.groups[0])
                new_mn.add_group(sec_network.groups[1])
            else:
                new_mn.add_group(sec_network.groups[0])
                new_mn.add_group(self.groups[1])
            return new_mn

        # Handle crossover in any other case.
        if self.crossover_type == 0:
            new_mn = self._random_crossover(sec_network,new_mn)
        else: # Note: Change to 1 if ever implementing other types of crossover.
            new_mn = self._two_pt_crossover(sec_network,new_mn)

        # Crossover the indirect connections
        # Indirect cons already obtained from self, only replace with sec_network.
        for i in xrange(len(self.indirect_cons)):
            if random.random() < 0.5:
                new_mn.indirect_cons[i] = sec_network.indirect_cons[i]

        return new_mn

    def get_raw_activations(self,inp,con_num=0):
        """ Get the raw activations for each muscle group.

        If group_num is specified then get only that node.  Otherwise, get the activations
        for all groups and return in a 2d list.

        Args:
            inp: input activation value
            con_num: which connection to get activations from.

        Returns:
            list containing either one or all activations.
        """
        if con_num:
            return self.groups[self.indirect_cons[con_num]].get_raw_activations(inp)
        else:
            act_list = []
            for con in self.indirect_cons:
                act_list.append(self.groups[con].get_raw_activations(inp))
            return act_list
 
    def get_activations(self,inp,con_num=0):
        """ Get the activations by axis for each muscle group.

        Args:
            con_num: connection number to get the activations of
            inp: input activation value

        Returns:
            list containing one or all activations
        """
        if con_num:
            xacts, yacts = self.groups[self.indirect_cons[con_num]].get_activations(inp)
            return [xacts,yacts]
        else:
            activations = []
            for con in self.indirect_cons:
                xacts, yacts = self.groups[con].get_activations(inp)
                activations.append([xacts,yacts])
            return activations

    def get_avg_activations(self,inp,con_num=0,inputs=0):
        """ Get the average activations for an entire muscle group.

        Args:
            con_num: only get this connection if specified
            inp: input value

        Returns:
            list of activations
        """
        if inputs != 0:
            acts = []
            for inp, con in zip(inputs,self.indirect_cons):
                x,y = self.groups[con].get_avg_activations(inp)
                acts.append([x,y])
            return acts
        else:
            if con_num:
                x,y = self.groups[self.indirect_cons[con_num]].get_avg_activations(inp)
                return [x,y]
            else:
                acts = []
                for con in self.indirect_cons:
                    x,y = self.groups[con].get_avg_activations(inp)
                    acts.append([x,y])
                return acts

class IndirectMirroringMuscleNetwork(IndirectMuscleNetwork):
    """ Class that includes indirection of the muscle groups and also allows for mirroring activations. """

    def __init__(self,num_groups=0,num_nodes=0,gid=-1,crossover=0,num_indirect=0,indirect_cons=[],mirror_vals=[]):
        """ Initialize a muscle network.

        Args:
            num_groups: number of muscle groups to randomly create.
            num_nodes: list of ints defining the number of nodes for each group
            gid: genome id of associated meta-controller (NEAT implmentation)
            crossover: type of crossover to use 0 - random 1 - two point
            num_indirect: number of indirect connections to connect to outputs
            indirect_cons: list to specify how the indirect connections should initially be made
            mirror_vals: how the joints should be mirrored
        """
        super(IndirectMirroringMuscleNetwork, self).__init__(num_groups,num_nodes,gid,crossover,num_indirect,indirect_cons)
        
        # Setup the mirror values.
        if mirror_vals:
            self.mirror_vals = mirror_vals
        else:
            self.mirror_vals = [0 for i in xrange(num_groups)]

    def __str__(self):
        ret = "<IndirectMirroringMuscleNetwork>\n"
        ret+= "<GID>"+str(self.gid)+"</GID>\n"
        ret += "<IndMap>"+str(', '.join(str(item) for item in self.indirect_cons))+"</IndMap>\n"
        ret += "<MirMap>"+str(', '.join(str(item) for item in self.mirror_vals))+"</MirMap>\n"
        for group in self.groups:
            ret += str(group)
        ret += "</IndirectMirroringMuscleNetwork>\n"
        return ret

    def copy(self):
        """ Copy the indirect mirroring muscle network into a new muscle network. """
        mn = IndirectMirroringMuscleNetwork(gid=self.gid,indirect_cons=self.indirect_cons,mirror_vals=self.mirror_vals)
        for group in self.groups:
            mg = group.copy()
            mn.groups.append(mg)
        return mn

    def mutate(self,mut_perc=0.025):
        """ Mutate the muscle network.

        Args:
            mut_perc: what the percentage of mutation should be. 
        """
        super(IndirectMirroringMuscleNetwork, self).mutate(mut_perc=mut_perc)

        # Mutate the mirroring values.
        for index,mir_val in enumerate(self.mirror_vals):
            if random.random() <= mut_perc:
                self.mirror_vals[index] = random.randint(0,3)

    def crossover(self,sec_network):
        """ Crossover two muscle networks.

        Crossover occurs at the muscle group level.  We therefore do not break the individual
        muscle groups up.

        Args:
            sec_network: network to perform crossover with.
        """
        # Ensure that the groups are the same size.
        if(len(self.groups) != len(sec_network.groups)):
            print("Networks not the same number of groups!")
            return self.copy()

        new_mn = IndirectMirroringMuscleNetwork(gid=self.gid,indirect_cons=self.indirect_cons,mirror_vals=self.mirror_vals)

        # Don't crossover if only one group.
        if len(self.groups) <= 1:
            if random.random() <= 0.5:
                return self.copy()
            else:
                return sec_network.copy()
        elif len(self.groups) == 2: # Handle only one position crossover.
            if random.random() <= 0.5:
                new_mn.add_group(self.groups[0])
                new_mn.add_group(sec_network.groups[1])
            else:
                new_mn.add_group(sec_network.groups[0])
                new_mn.add_group(self.groups[1])
            return new_mn

        # Handle crossover in any other case.
        if self.crossover_type == 0:
            new_mn = self._random_crossover(sec_network,new_mn)
        else: # Note: Change to 1 if ever implementing other types of crossover.
            new_mn = self._two_pt_crossover(sec_network,new_mn)

        # Crossover the indirect connections
        # Indirect cons already obtained from self, only replace with sec_network.
        for i in xrange(len(self.indirect_cons)):
            if random.random() < 0.5:
                new_mn.indirect_cons[i] = sec_network.indirect_cons[i]

        # Crossover the mirror values
        # Mirror vals already obtained from self, only replace with sec_network.
        for i in xrange(len(self.mirror_vals)):
            if random.random() < 0.5:
                new_mn.mirror_vals[i] = sec_network.mirror_vals[i]

        return new_mn

    def get_raw_activations(self,inp,con_num=0):
        """ Get the raw activations for each muscle group.

        # TODO: don't really need this implemented.

        If group_num is specified then get only that node.  Otherwise, get the activations
        for all groups and return in a 2d list.

        Args:
            inp: input activation value
            con_num: which connection to get activations from.

        Returns:
            list containing either one or all activations.
        """
        return 0
        #if con_num:
        #    return self.groups[self.indirect_cons[con_num]].get_raw_activations(inp)
        #else:
        #    act_list = []
        #    for con in self.indirect_cons:
        #        act_list.append(self.groups[con].get_raw_activations(inp))
        #    return act_list

    def get_activations(self,inp,con_num=0):
        """ Get the activations by axis for each muscle group.

        # TODO: Don't need to implement this.

        Args:
            con_num: connection number to get the activations of
            inp: input activation value

        Returns:
            list containing one or all activations
        """
        return 0
        #if con_num:
        #    xacts, yacts = self.groups[self.indirect_cons[con_num]].get_activations(inp)
        #    return [xacts,yacts]
        #else:
        #    activations = []
        #    for con in self.indirect_cons:
        #        xacts, yacts = self.groups[con].get_activations(inp)
        #        activations.append([xacts,yacts])
        #    return activations

    def get_avg_activations(self,inp,con_num=0,inputs=0):
        """ Get the average activations for an entire muscle group.

        Args:
            con_num: only get this connection if specified
            inp: input value

        Returns:
            list of activations
        """
        activations = super(IndirectMirroringMuscleNetwork, self).get_avg_activations(inp,con_num,inputs)

        # Querying for a specific muscle group.
        if con_num:
            if self.mirror_vals[con_num] == 0: # No Change to Values
                pass
            elif self.mirror_vals[con_num] == 1: # Flip the first activation signal
                activations[0] *= -1
            elif self.mirror_vals[con_num] == 2: # Flip the second activation signal
                activations[1] *= -1
            elif self.mirror_vals[con_num] == 3: # Flip both activation signals.
                activations[0] *= -1
                activations[1] *= -1
            return activations
        else: # Bulk processing all groups at once.
            for index, act in enumerate(activations):
                if self.mirror_vals[index] == 0: # No Change to Values
                    pass
                elif self.mirror_vals[index] == 1: # Flip the first activation signal
                    activations[index][0] *= -1
                elif self.mirror_vals[index] == 2: # Flip the second activation signal
                    activations[index][1] *= -1
                elif self.mirror_vals[index] == 3: # Flip both activation signals.
                    activations[index][0] *= -1
                    activations[index][1] *= -1
            return activations