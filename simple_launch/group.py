from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from .events import When


class Group:

    def __init__(self, ns=None, condition=None, container='',
                 parent = None,
                 when: When = None):

        self.__parent = parent
        self.__actions = []
        self.__condition = condition
        self.__container = container
        self.__when = when

        if ns is not None:
            self.add_action(PushRosNamespace(ns))

    def is_container(self):
        return self.__container

    def close(self):

        if self.__parent is None:
            # main group
            return self.__actions

        if self.__container:
            # actions (e.g. ComposableNodes) are added by SimpleLauncher when creating the container
            return self.__parent, self.__actions

        # closing a classical group, potentially with event handling
        if self.__actions:
            group = GroupAction(self.__actions, condition=self.__condition)
            if self.__when is not None:
                self.__parent.add_action(self.__when.wrap(group))
            else:
                self.__parent.add_action(group)

        return self.__parent

    def add_action(self, action):

        if isinstance(action, list):
            self.__actions.extend(action)
        else:
            self.__actions.append(action)
