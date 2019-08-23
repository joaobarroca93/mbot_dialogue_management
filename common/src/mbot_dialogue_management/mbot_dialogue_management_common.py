#!/usr/bin/env python

from __future__ import print_function

import json
import copy
import yaml
import rospy

def print_slots(slots_dict):
	for slot in slots_dict:
		if not slots_dict[slot] is None:
			print("{}->{}".format(slot, slots_dict[slot]))


class Slot(object):

	def __init__(self, type, value=None, confidence=0.0):

		self.type = type
		self.value = value
		self.confidence = confidence

	def as_dict(self):
		return {
			"type": self.type,
			"value": self.value,
			"confidence": self.confidence
		}

class DialogueAct(object):

	def __init__(self, dtype, slots=[], confidence=0.0):

		self.dtype = dtype
		self.slots = slots
		self.confidence = confidence

	def as_dict(self):
		return {
			"dtype": self.dtype,
			"slots": [{
				"type": slot.type,
				"value": slot.value,
				"confidence": slot.confidence}
			 for slot in self.slots],
			 "confidence": self.confidence
		}

	def as_system_response(self):

		if not self.slots:
			return ''.join([self.dtype, "()"])

		elif len(self.slots) == 1:
			if self.slots[0].value:
				return ''.join([self.dtype, "(", self.slots[0].type, "=", self.slots[0].value, ")"])
			else:
				return ''.join([self.dtype, "(", self.slots[0].type, ")"])



class DialogueState(object):

	def __init__(self, slots=[]):

		self.slots = slots

	def as_dict(self):
		return [{"type": slot.type,
				"value": slot.value,
				"confidence": slot.confidence}
				for slot in self.slots]



class DialogueManagement(object):

	def __init__(self, task_ontology_path, task_threshold=0.7,
				slot_threshold=0.7, request_threshold=0.1):

		self.task_threshold = task_threshold
		self.slot_threshold = slot_threshold
		self.request_threshold = request_threshold

		with open(task_ontology_path, "r") as f:
			self.task_ontology = json.load(f)
			self.task_ontology = yaml.safe_load(
				json.dumps(self.task_ontology)
			)

	@staticmethod
	def find_slots(dialogue_state, type):
		return [{"value": slot.value, "confidence": slot.confidence} for slot in dialogue_state.slots if slot.type==type]

	@staticmethod
	def generate_response(dtype, slots=[]):
		return DialogueAct(dtype=dtype, slots=slots)

	@staticmethod
	def check_task_req(task, requirement, dialogue_state, threshold=0.2):

		#print("Checking required slots for task <{}>".format(task))

		missing = 0
		missing_slots = []
		relevant_slots = []
		for req_slot in requirement["required slots"]:

			#print(req_slot, end="->")

			try:
				# check if the required slot is already in the dialogue state
				assert req_slot in [slot.type for slot in dialogue_state.slots]
				#print("CHECK")

				max_conf = 0.0
				relevant_slot = None
				for slot in dialogue_state.slots:
					#print(slot.as_dict())
					if slot.type == req_slot and slot.confidence > max_conf and slot.confidence > threshold:
						#print("{} is relevant".format(slot.type))
						max_conf = slot.confidence

						relevant_slots.append({
							"type": slot.type,
							"value": slot.value,
							"confidence": slot.confidence
						})

					elif slot.type == req_slot and slot.confidence < threshold:
						#print(slot.confidence, threshold)
						raise AssertionError()

			except AssertionError:
				#print("MISSING")
				missing += 1
				missing_slots.append(req_slot)

		relevant_slots = sorted(
			relevant_slots,
			key=lambda x: x["confidence"], reverse=True
		)

		return missing_slots, relevant_slots

	def run(self, dialogue_state):

		tasks = self.find_slots(dialogue_state, type="intent")
		system_response= None
		task = None
		task_conf = 0.0
		task_obj = None
		slots_missing = []
		relevant_slots = []
		relevant_slot_confirm = False

		rospy.logdebug(tasks)

		if not dialogue_state.slots:
			system_response = self.generate_response(dtype="hello", slots=[])

		elif not tasks:
			system_response = self.generate_response(dtype="request", slots=[Slot(type="intent")])

		else:
			tasks = sorted(tasks, key=lambda x: x["confidence"])
			task_conf = tasks[-1]["confidence"]

			if task_conf < self.task_threshold:
				system_response = self.generate_response(dtype="confirm", slots=[Slot(type="intent", value=tasks[-1]["value"])])
			else:
				task = tasks[-1]["value"]
	
		if task:

			rospy.logdebug("task={}".format(task))

			task_reqs = self.task_ontology[task]

			required_slots = []
			min_missing = 100
			for req in task_reqs:
				relevant_slot_confirm = False
				slots_missing, relevant_slots = self.check_task_req(task, req, dialogue_state, threshold=self.request_threshold)

				for slot in relevant_slots:
					if slot["confidence"] < self.slot_threshold:
						relevant_slot_confirm = True
						system_response = self.generate_response(dtype="confirm", slots=[Slot(type=slot["type"], value=slot["value"])])

				if len(slots_missing) < min_missing:
					min_missing = len(slots_missing)
					required_slots = slots_missing

			if required_slots and not system_response:
				rospy.logdebug("required_slots={}".format(required_slots))
				slot = required_slots[0]
				rospy.logdebug("requiring {}".format(slot))
				system_response = self.generate_response(dtype="request", slots=[Slot(type=slot)])

		if not slots_missing and relevant_slots and not relevant_slot_confirm:
			rospy.logdebug("All requirements fullfield to perform the task <{}>".format(task))
			rospy.logdebug(relevant_slots)
			system_response = self.generate_response(dtype="bye", slots=[])

			task_obj = DialogueState(
				slots=[
					Slot(
						type=slot["type"],
						value=slot["value"],
						confidence=slot["confidence"]
					)
				for slot in relevant_slots]
			)
			task_obj.slots.append(Slot(type="intent", value=task, confidence=task_conf))

		return system_response, task_obj

if __name__ == '__main__':

	dialogue_state = DialogueState(
		slots=[
			Slot(type="intent", value="take", confidence=0.9997860789299011),
			Slot(type="person", value="me", confidence=0.9999368243825574),
			Slot(type="object", value="book", confidence=0.9994227108432334),
			Slot(type="source", value="tv table", confidence=0.9979189738917265),
			Slot(type="object", value="tv table", confidence=0.4447823174986588),
			Slot(type="destination", value="tv table", confidence=0.07724273892602987),
		]
	)

	dm_obj = DialogueManagement("task_ontology.json")

	system_response = dm_obj.run(dialogue_state)
	
	if system_response:
		print("\nSYSTEM RESPONSE:", end=' ')
		print( json.dumps(system_response.as_system_response(), indent=4) )

