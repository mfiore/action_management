#!/usr/bin/env python

import sys
import rospy
import action_management_msgs.srv
import common_msgs.msg
def checkPreconditions(action_name,action_parameters):
	try:
		service_name="/action_management/actions/"+action_name+"/checkPreconditions"
		print "waiting for service name "+service_name
		rospy.wait_for_service(service_name)
		print "connected"
		check_preconditions_srv=rospy.ServiceProxy(service_name,
			action_management_msgs.srv.CheckPreconditions)
		res=check_preconditions_srv(action_parameters)
		print "result is "+str(res.value)
	except Exception, e:
		print "Fail in checking preconditions"
		print type(e)     # the exception instance
		print e           # __str__ allows args to be printed directly
	else:
		pass
	finally:
		pass

if __name__ == '__main__':
	rospy.init_node("action_nodes_utilities")
	action_name=sys.argv[1]
	parameter_list=common_msgs.msg.ParameterList()
	parameters=[]
	i=2
	while i<len(sys.argv):
		parameter=common_msgs.msg.Parameter(sys.argv[i],sys.argv[i+1])
		parameters.append(parameter)
		i=i+2
	parameter_list.parameter_list=parameters
	checkPreconditions(action_name,parameter_list)