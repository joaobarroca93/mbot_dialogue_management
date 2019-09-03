#!/usr/bin/env python

import rospy
import rospkg

import json
import os

from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueManagement
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import Slot as MbotSlot
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueAct as MbotDialogueAct
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueState as MbotDialogueState

from mbot_dialogue_management.msg import (InformSlot, DialogAct, DialogState)
from std_msgs.msg import String


"""
Description: This function helps logging parameters as debug verbosity messages.

Inputs:
	- param_dict: a dictionary whose keys are the parameter's names and values the parameter's values.
"""
def logdebug_param(param_dict):
	[ rospy.logdebug( '{:<20}\t{}'.format(param[0], param[1]) ) for param in param_dict.items() ]



class DMNode(object):

	def __init__(self, debug=False):

		# get useful parameters, and if any of them doesn't exist, use the default value
		rate 			= rospy.get_param('~loop_rate', 10.0)
		node_name 		= rospy.get_param('~node_name', 'dialogue_management')
		d_state_topic 	= rospy.get_param('~d_state_topic_name', '/dialogue_state')
		system_response = rospy.get_param('~system_response_topic_name', '/system_response')
		dialogue_status = rospy.get_param('~dialogue_status_topic_name', '/dialogue_status')
		task 			= rospy.get_param('~task_topic_name', '/task')
		ontology_path 	= rospy.get_param('~ontology_path', 'common/src/mbot_dialogue_management/task_ontology.json')

		# initializes the node (if debug, initializes in debug mode)
		if debug == True:
			rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
			rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
		else:
			rospy.init_node(node_name, anonymous=False)
			rospy.loginfo("%s node created" % node_name)

		# set parameters to make sure all parameters are set to the parameter server
		rospy.set_param('~loop_rate', rate)
		rospy.set_param('~node_name', node_name)
		rospy.set_param('~d_state_topic_name', d_state_topic)
		rospy.set_param('~system_response_topic_name', system_response)
		rospy.set_param('~dialogue_status_topic_name', dialogue_status)
		rospy.set_param('~ontology_full_name', ontology_path)
		rospy.set_param('~task_topic_name', task)

		#rospy.logdebug('=== NODE PRIVATE PARAMETERS ============')
		#logdebug_param(rospy.get_param(node_name))

		rospack = rospkg.RosPack()
		# get useful paths
		generic_path 	= rospack.get_path("mbot_dialogue_management")
		ontology_path 	= os.path.join(generic_path, ontology_path)
		logdebug_param({'generic_path': generic_path, 'ontology_full_path': ontology_path})

		#slots = ["object", "person", "destination", "source"]
		slots = ["object"]
		scirob = True
		self.dm_object = DialogueManagement(
			ontology_path, task_threshold=0.7, slot_threshold=0.2, slots=slots, scirob=scirob
		)
		rospy.loginfo('dialogue management object created')

		self.dm_request_received = False
		self.loop_rate = rospy.Rate(rate)
		self.last_system_response = None
		self.dialogue_state = None

		rospy.Subscriber(d_state_topic, DialogState, self.dmCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", d_state_topic)

		self.pub_system_response = rospy.Publisher(system_response, DialogAct, queue_size=1)
		self.pub_task = rospy.Publisher(task, DialogState, queue_size=1)
		self.pub_dialogue_status = rospy.Publisher(dialogue_status, String, queue_size=1)
		
		rospy.loginfo("%s initialization completed! Ready to accept requests" % node_name)

	def dmCallback(self, dialogue_state_msg):

		rospy.loginfo('[Message received]')
		rospy.logdebug('{}'.format(dialogue_state_msg))

		self.dm_request_received = True
		self.dialogue_state = dialogue_state_msg


	def begin(self):

		THRESHOLD = 0.1

		while not rospy.is_shutdown():

			if self.dm_request_received == True:

				rospy.loginfo('[Handling message]')
				self.dm_request_received = False

				dialogue_state = MbotDialogueState(
					slots=[MbotSlot(
						type=slot.slot,
						value=slot.value,
						confidence=slot.confidence
					) for slot in self.dialogue_state.slots]
				)

				rospy.logdebug('dialogue_state_dict: {}'.format(dialogue_state.as_dict()))

				rospy.loginfo('[Computing system response]')
				system_response, task = self.dm_object.run(dialogue_state)

				if system_response:
					rospy.loginfo('system response: {}'.format(system_response.as_system_response()))

					system_response_msg = DialogAct()
					system_response_msg.dtype = system_response.dtype
					system_response_msg.slots = [
						InformSlot(
							slot=slot.type,
							value=slot.value,
							confidence=slot.confidence,
							known=True
						)
					for slot in system_response.slots]
					
					self.pub_system_response.publish(system_response_msg)

					if system_response_msg.dtype == "bye":
						self.pub_dialogue_status.publish(String("finish"))
					elif system_response_msg.dtype == "hello":
						self.pub_dialogue_status.publish(String("begin"))
					else:
						self.pub_dialogue_status.publish(String("continue"))

				if task:
					rospy.loginfo('task: {}'.format(task.as_dict()))
					task_msg = DialogState()
					task_msg.slots = [
						InformSlot(
							slot=slot.type,
							value=slot.value,
							confidence=slot.confidence,
							known=True
						)
					for slot in task.args]

					self.pub_task.publish(task_msg)
				
			self.loop_rate.sleep()


def main():

	dm_node = DMNode(debug=True)
	dm_node.begin()
