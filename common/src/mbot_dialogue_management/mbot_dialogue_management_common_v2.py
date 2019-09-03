#!/usr/bin/env python

from __future__ import print_function

import json
import copy
import yaml
import rospy

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
		else:
			string = ''.join([self.dtype, "("])
			for slot in self.slots:
				string = ''.join([string, slot.type, "=", slot.value, ","])
			return ''.join([string, ")"])


class DialogueState(object):

	def __init__(self, slots=[]):

		self.slots = slots

	def as_dict(self):
		return [{"type": slot.type,
				"value": slot.value,
				"confidence": slot.confidence}
				for slot in self.slots]

class Task(object):

	def __init__(self, intent_slot, task_req):

		self.type = intent_slot.value
		self.confidence = intent_slot.confidence
		self.task_req = task_req["required slots"]
		self.task_allowed = task_req["allowed slots"]
		self.task_conf = []
		self.args = []
		self.args.append(intent_slot)

	def add_arg(self, arg):
		# add argument if it is allowed
		if arg.type in self.task_allowed:
			self.args.append(arg)
			# remove the argument from task requirements
			if arg.type in self.task_req:
				self.task_req.remove(arg.type)
			# update the confidence by multiplying the confidence of the new argument
			self.confidence *= arg.confidence
			self.args = sorted(self.args, key=lambda x: x.confidence, reverse=True)
			return True
		return False

	def as_dict(self):
		return {
			"type": self.type,
			"confidence": self.confidence,
			"args": [{
				"type": slot.type,
				"value": slot.value,
				"confidence": slot.confidence}
			 for slot in self.args],
			 "required_args": self.task_req,
			 "allowed_args": self.task_allowed
		}


class DialogueManagement(object):

	def __init__(self, task_ontology_path, slots, task_threshold=0.7,
				slot_threshold=0.2, scirob=False):

		self.task_threshold = task_threshold
		self.slot_threshold = slot_threshold

		self.slots = slots
		self.scirob = scirob

		with open(task_ontology_path, "r") as f:
			self.task_ontology = json.load(f)
			self.task_ontology = yaml.safe_load(
				json.dumps(self.task_ontology)
			)

	@staticmethod
	def find_slots(dialogue_state, type, threshold):
		return [{"type": type, "value": slot.value, "confidence": slot.confidence}
		for slot in dialogue_state.slots if slot.type==type and slot.confidence > threshold]

	@staticmethod
	def generate_response(dtype, slots=[]):
		return DialogueAct(dtype=dtype, slots=slots)

	@staticmethod
	def fill_tasks(tasks, slots):
			filled_tasks = []
			for task in tasks:
				for slot in slots:
					filled_task = copy.deepcopy(task)
					filled_task.add_arg(Slot(type=slot["type"], value=slot["value"], confidence=slot["confidence"]))
					filled_tasks.append(filled_task)
			return copy.deepcopy(filled_tasks)

	def run(self, dialogue_state):

		system_response= None
		task = None
		tasks = None
		slots_missing = None
		relevant_slots = None

		# Check if the dialogue state has slots if the dype intent, to indentify the task
		intent_slots = self.find_slots(dialogue_state, type="intent", threshold=self.slot_threshold)

		if self.scirob:
			intent_slots = [{"type": "intent", "value": "order", "confidence": 1.0}]

		if not dialogue_state.slots:
			system_response = self.generate_response(dtype="hello", slots=[])
		
		# if there is not any task, generate a request intent system response
		elif not intent_slots:
			system_response = self.generate_response(dtype="request", slots=[Slot(type="intent")])

		else:

			try:
				tasks = [
					Task(Slot(intent_slot["type"], intent_slot["value"], intent_slot["confidence"]), task_req)
				for intent_slot in intent_slots for task_req in self.task_ontology[intent_slot["value"]] if intent_slot["confidence"]]

			except KeyError:
				raise KeyError("Intent value not in the specified task requirements")


			# fill tasks
			slots_in_state = [self.find_slots(dialogue_state, type=slot, threshold=self.slot_threshold) for slot in self.slots]
			if slots_in_state:

				if self.scirob:
					slots_in_state = [[slot] for slot in slots_in_state[0]]
				
				for slots in slots_in_state:
					if slots:
						tasks = self.fill_tasks(tasks, slots)

			# sort tasks by confidence score (we want the highest to be first on the list)
			tasks = sorted(tasks, key=lambda x: x.confidence, reverse=True)
			# and then sort them by the missing arguments (we want first the tasks with fewer missing arguments)
			tasks = sorted(tasks, key=lambda x: len(x.task_req))

		if tasks:

			# choose the highest confident task
			task = tasks[0]

			#print(json.dumps(task.as_dict(), indent=4))

			# if it is not confident enough, ask for confirmation of task
			if task.confidence < self.task_threshold:
				system_response = self.generate_response(dtype="confirm", slots=task.args)

			# else, check if there are arguments missing
			elif task.task_req:
					system_response = self.generate_response(dtype="request", slots=[Slot(type=task.task_req[0])])
			
			# all arguments are filled, ready to perform task
			else:
				#print("All requirements fullfield to perform the task")
				#print(json.dumps(task.as_dict(), indent=4))
				system_response = self.generate_response(dtype="bye", slots=[])
				return (system_response, task)

		return (system_response, None)


if __name__ == '__main__':

	dialogue_state = DialogueState(
		slots=[
			Slot(type="intent", value="take", confidence=0.9997860789299011),
			Slot(type="person", value="me", confidence=0.9999368243825574),
			Slot(type="object", value="beer", confidence=0.9994227108432334),
			Slot(type="source", value="fridge", confidence=0.9979189738917265),
			Slot(type="object", value="fridge", confidence=0.09631608460764922),
			Slot(type="person", value="fridge", confidence=0.02914276224060807),
		]
	)

	dm_obj = DialogueManagement("task_ontology.json")

	system_response, _ = dm_obj.run(dialogue_state)
	
	if system_response:
		rospy.loginfo("\nSYSTEM RESPONSE:")
		rospy.loginfo( json.dumps(system_response.as_system_response(), indent=4) )

