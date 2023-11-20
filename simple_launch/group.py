from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch import Action
from .events import When


class Group:

    def __init__(self, ns=None, condition=None, container='',
                 parent = None,
                 when: When = None):

        self.__parent = parent
        self.__condition = condition
        self.__container = container
        self.__when = when

        # inherit namespace from this branch
        ns = [ns] if ns is not None else []
        if parent is None:
            self.__root = self
            self.__ns = ns
        else:
            self.__root = parent.__root
            self.__ns = parent.__ns + ns

        # actions are always active and added to the root group
        self.__actions = []
        # managed are added as Event or Containers
        self.__managed = []

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

            if any(not isinstance(action, Action) for action in self.__actions):
                # probably a raw function call due to OnProcessIO
                # skip GroupAction as it makes it not callable
                group = self.__actions
            else:
                ns_tree = list(map(PushRosNamespace, self.__ns))
                group = GroupAction(ns_tree + self.__actions, condition=self.__condition)
            if self.__when is not None:
                self.__root.add_action(self.__when.register(group))
            else:
                self.__root.add_action(group)

        return self.__parent

    def add_action(self, action):

        if isinstance(action, list):
            self.__actions.extend(action)
        else:
            self.__actions.append(action)
