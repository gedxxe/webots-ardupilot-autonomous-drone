"""Mission-level autonomy state machines.

Submodules are intentionally not imported here. Importing `mission` pulls in
control modules, while control modules also need `autonomy.commands`; eager
imports here would create a circular dependency.
"""
