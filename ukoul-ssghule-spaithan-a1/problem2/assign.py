#!/usr/bin/env python

'''
ABSTRACTION
Initial state: Initial state is the collection of all groups as preferred by the students
Set of valid states: A collection of groups where there is minimal conflict between groups
The successor function: No successor function defined, a local search where pairwise combination of all grups considered
Cost function: A function that calculates the cost of grading basis values of k, m and n
Goal state: A collection of groups where all students are assigned exactly one group with size of less than or equal to 3 

'''

import sys
import pandas as pd
import itertools
import collections
import random

def calc_cost(groups):
        grade_cost=len(groups)
#        print len(groups)
        group_cost=0
        pref_cost=0
        unpref_cost=0
        for i in groups:
#                print i
                for j in i:
#                        print j
                        if(any(data.gsize[data['sid']==j]==0)):
                                group_cost=group_cost=group_cost+0
#                                print('gc0 ', group_cost)
                        elif(any(len(i)!=data.gsize[data['sid']==j])):
                                group_cost=group_cost+1
#                                print('gc1 ', group_cost)
                        if(any(data.pref[data['sid']==j]=='_')):
                                pref_cost=pref_cost+0
#                                print('pc0 ',pref_cost)
                        else:
                                for p in (data.pref[data['sid']==j].tolist()[0].split(",")):
                                        if (p not in i):
                                                pref_cost=pref_cost+1
#                                print('pc1 ', pref_cost)
                        if(any(data.unpref[data['sid']==j]=='_')):
                                unpref_cost=unpref_cost+0
#                                print('un0 ', unpref_cost)
                        else:
                                for u in (data.unpref[data['sid']==j].tolist()[0].split(",")):
                                        if (u in i):
                                                unpref_cost=unpref_cost+1
#                                print('up1 ', unpref_cost)
        return (k*grade_cost+group_cost+n*pref_cost+m*unpref_cost)

def check_duplicates(groups):
        all_members=[member for group in groups for member in group]
        count=collections.Counter(all_members).values()
        return any(c>1 for c in count)

def is_goal(groups):
        all_students=[i for i in data['sid']]
        all_members=[member for group in groups for member in group]
        return all(student in all_members for student in all_students) and not check_duplicates(groups)

def getmaxcost(groups):
	cost=[calc_cost([i]) for i in groups]
	return groups, groups.pop(cost.index(max(cost)))

def remove_duplicates(group1, group2):
	dupl=[member for member in group1 if member in group2]
	for i in dupl:
		group1.pop(group1.index(i))
		group2.pop(group2.index(i))
		if(calc_cost([group1])<calc_cost([group2])):
			group2.append(i)
		else:
			group1.append(i)
	return group1, group2	
		

def solve(groups):
	for i in groups[:]:
		for j in groups[:]:
			if(check_duplicates([i,j]) and groups.index(i)!=groups.index(j)):
				groups.pop(groups.index(i))
				groups.pop(groups.index(j))
				if(len(remove_duplicates(i,j)[0])!=0):
					groups.append(remove_duplicates(i,j)[0])
				if(len(remove_duplicates(i,j)[1])!=0):
					groups.append(remove_duplicates(i,j)[1])	

		if(is_goal(groups)):
			return groups
               			        
def printable(groups):
	return "\n".join([ " ".join([member for member in group]) for group in groups])


fname = str(sys.argv[1])
k = int(sys.argv[2])
m = int(sys.argv[3])
n= int(sys.argv[4])

data = pd.read_csv(fname, header = None, delimiter=" ")
data.columns=['sid','gsize','pref','unpref']

initial_groups=[]

for i in data['sid']:
	if(any(data.pref[data['sid']==i]!='_')):
		group=data.pref[data['sid']==i].tolist()[0].split(",")
	else:
		group=[]
	group.append(i)
	initial_groups.append(group)

print printable(solve(initial_groups))
print calc_cost(solve(initial_groups))
