from __future__ import annotations # allows referencing Vector2 from inside Vector2. Will be unnecessary in the future
from typing import List, Callable
from enum import Enum
from abc import ABC, abstractmethod
import numpy as np
from utils import Vector2

class VehicleType(Enum):
	civilian = 0
	emergency = 1

class ContinuousVehicle(ABC):
	_vehicle_type: VehicleType
	_position: Vector2
	_velocity: Vector2
	_other_vehicles: List[ContinuousVehicle]
	_position_is_obstacle: Callable[[Vector2], bool]
	_heading: float # between 0 and 2pi with 0 being positive y direction, pi / 2 positive x direction

	def __init__(self, vehicle_type: VehicleType, position: Vector2, velocity: Vector2, heading: float = 0):
		self._vehicle_type = vehicle_type
		self._position = position
		self._velocity = velocity
		self._other_vehicles = []
		self._heading = heading

	@property
	def vehicle_type(self): return self._vehicle_type

	@property
	def position(self): return self._position

	@property
	def velocity(self): return self._velocity

	@property
	def heading(self): return self._heading

	@heading.setter
	def heading(self, value: float): self._heading = value % (2 * np.pi)

	def _roll_forward(self, dt: float):
		self._position += self._velocity * dt

	def add_vehicle(self, vehicle: ContinuousVehicle):
		assert self is not vehicle
		for v in self._other_vehicles:
			assert v is not vehicle
		self._other_vehicles.append(vehicle)

	def remove_vehicle(self, vehicle: ContinuousVehicle):
		remove_index: None | int = None
		for index, v in enumerate(self._other_vehicles):
			if v is vehicle:
				remove_index = index
				break
		assert remove_index is not None
		del self._other_vehicles[remove_index]

	@abstractmethod
	def update_velocity(self):
		"""
		This function should update the velocity property based on the information the vehicle holds.
		"""
		raise NotImplementedError

	@abstractmethod
	def contains(self, position: Vector2) -> bool:
		"""
		This function should return whether the given position lies within this vehicle.
		"""
		raise NotImplementedError

