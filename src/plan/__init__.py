"""
Charlie Robot - Plan Module
Behavior Tree + ROS2 per navigazione e decisioni.
"""

# Export per uso esterno
from .plan_node import PlanNode, build_behavior_tree

__all__ = [
    "PlanNode",
    "build_behavior_tree",
]
