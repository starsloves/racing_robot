"""Compatibility shim: enhanced_return_navigator → stage3_return_navigator."""

from racing_stage3.stage3_return_navigator import Stage3ReturnNavigator as EnhancedReturnNavigator
from racing_stage3.stage3_return_navigator import main

__all__ = ['EnhancedReturnNavigator', 'main']
