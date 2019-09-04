#!/usr/bin/env python

import rospy
import rospkg

import json
import os
import yaml

from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueManagement
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import Slot as MbotSlot
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueAct as MbotDialogueAct
from mbot_dialogue_management.mbot_dialogue_management_common_v2 import DialogueState as MbotDialogueState

from mbot_dialogue_management.msg import (InformSlot, DialogAct, DialogState)
from std_msgs.msg import String


class DMNode(object):

	def __init__(self):

		rospack = rospkg.RosPack()
		generic_path 	= rospack.get_path("mbot_dialogue_management")

		try:
			# need to download a .yaml config with all the needed parameters !
			rospy.loginfo("Loading node config")
			config_path = rospy.myargv()[1]
			config = yaml.load(open(config_path))
		except IndexError:
			rospy.loginfo("Loading node config")
			config_path = os.path.join(generic_path, "ros/config/config_mbot_dialogue_management.yaml")
			config = yaml.load(open(config_path))

		node_name 				= config["node_params"]["name"]
		rate 					= config["node_params"]["rate"]
		debug 					= config["node_params"]["debug"]
		task_topic 				= config["node_params"]["task_topic"]
		d_state_topic 			= config["node_params"]["d_state_topic"]
		d_acts_topic 			= config["node_params"]["d_acts_topic"]
		dialogue_status_topic	= config["node_params"]["dialogue_status_topic"]
		system_response_topic 	= config["node_params"]["system_response_topic"]
		ontology_path			= config["node_params"]["ontology_path"]
		slots					= config["node_params"]["slots"]
		scirob					= config["node_params"]["scirob"]
		task_threshold 			= config["node_params"]["task_threshold"]
		slot_threshold 			= config["node_params"]["slot_threshold"]

		# initializes the node (if debug, initializes in debug mode)
		if debug == True:
			rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
			rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
		else:
			rospy.init_node(node_name, anonymous=False)
			rospy.loginfo("%s node created" % node_name)

		ontology_path 	= os.path.join(generic_path, ontology_path)

		self.dm_object = DialogueManagement(
			ontology_path, task_threshold=task_threshold, slot_threshold=slot_threshold,
			slots=slots, scirob=scirob
		)
		rospy.loginfo('dialogue management object created')

		self.dm_request_received = False
		self.loop_rate = rospy.Rate(rate)
		self.last_system_response = None
		self.dialogue_state = None

		rospy.Subscriber(d_state_topic, DialogState, self.dmCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", d_state_topic)

		self.pub_system_response = rospy.Publisher(system_response_topic, DialogAct, queue_size=1)
		self.pub_task = rospy.Publisher(task_topic, DialogState, queue_size=1)
		self.pub_dialogue_status = rospy.Publisher(dialogue_status_topic, String, queue_size=1)
		
		rospy.loginfo("%s initialization completed! Ready to accept requests" % node_name)

	def dmCallback(self, dialogue_state_msg):

		rospy.loginfo('[Message received]')
		rospy.logdebug('{}'.format(dialogue_state_msg))

		self.dm_request_received = True
		self.dialogue_state = dialogue_state_msg


	def begin(self):

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

	dm_node = DMNode()
	dm_node.begin()
