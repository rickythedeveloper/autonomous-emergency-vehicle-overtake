from __future__ import annotations
from dataclasses import dataclass
from typing import List
import numpy as np
from .Pose import Pose
from .heading import clean_heading, subtract_heading
from .Circle import Circle
from ....utils.Vector2 import Vector2

@dataclass
class Arc:
	circle: Circle
	_start_angle: float
	arc_angle: float

	def __init__(self, circle: Circle, start_angle: float, arc_angle: float):
		self.circle = circle
		self.start_angle = start_angle
		self.arc_angle = arc_angle

	@property
	def start_angle(self): return self._start_angle

	@start_angle.setter
	def start_angle(self, value: float): self._start_angle = clean_heading(value)

	@property
	def end_angle(self):
		return clean_heading(self.start_angle + self.arc_angle)

	@property
	def start_heading(self):
		return clean_heading(self.start_angle + (1 if self.is_clock_wise else -1) * np.pi / 2)

	@property
	def end_heading(self):
		return clean_heading(self.end_angle + (1 if self.is_clock_wise else -1) * np.pi / 2)

	@property
	def start_position(self) -> Vector2:
		return Vector2(np.sin(self.start_angle), np.cos(self.start_angle)) * self.circle.radius + self.circle.center

	@property
	def end_position(self) -> Vector2:
		end_angle = self.end_angle
		return Vector2(np.sin(end_angle), np.cos(end_angle)) * self.circle.radius + self.circle.center

	@property
	def is_clock_wise(self): return self.arc_angle > 0

	@property
	def length(self): return np.abs(self.circle.radius * self.arc_angle)

	def split(self, num_segments: int) -> List[Arc]:
		arcs: List[Arc] = []
		segment_angle = self.arc_angle / num_segments
		for start_angle in np.arange(self.start_angle, self.start_angle + self.arc_angle, segment_angle):
			arcs.append(Arc(self.circle, start_angle, segment_angle))
		return arcs

def make_arc_from_origin(goal_position: Vector2) -> Arc:
	perp_bisector_slope = -1 / goal_position.slope
	perp_bisector_point = goal_position / 2
	perp_bisector_y_intercept = perp_bisector_point.y - perp_bisector_slope * perp_bisector_point.x # y = ax + b -> b = y - ax
	y_center = 0
	x_center = -perp_bisector_y_intercept / perp_bisector_slope # y = ax + b so x = -b/a
	center = Vector2(x_center, y_center)
	radius = np.abs(x_center)
	radius2 = (goal_position - center).length
	assert radius * 0.99 < radius2 < radius * 1.01, f'goal: {goal_position}, radius1={radius}, radius2={radius2}'
	circle = Circle(center, radius)

	start_angle = (3 / 2 if goal_position.x > 0 else 1 / 2) * np.pi
	is_clock_wise = goal_position.x > 0

	end_angle = circle.center.heading_to(goal_position)
	arc_angle = subtract_heading(end_angle, start_angle, 0 if is_clock_wise else -2 * np.pi)

	return Arc(circle, start_angle, arc_angle)

def make_arc(start_pose: Pose, goal_position: Vector2) -> Arc:
	relative_position = start_pose.position_world_to_relative(goal_position)
	arc_local_frame = make_arc_from_origin(relative_position)
	center = start_pose.position_relative_to_world(arc_local_frame.circle.center)
	circle = Circle(center, arc_local_frame.circle.radius)

	start_angle = clean_heading(arc_local_frame.start_angle + start_pose.heading)
	arc = Arc(circle, start_angle, arc_local_frame.arc_angle)
	return arc