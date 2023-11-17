from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete, OnProcessIO, OnShutdown, OnIncludeLaunchDescription
from . import console


def wrap(action, delay):
    if delay is None:
        return [action]
    if isinstance(delay, int):
        delay = float(delay)
    return [TimerAction(period=delay, actions=[action])]


class When:

    def __init__(self,
                 event,
                 action,
                 delay = None,
                 io = None):
        self.__ref = action
        self.__delay = delay
        self.__event = event

        if event == OnProcessExit:
            self.__arg = 'on_exit'
        elif event == OnProcessStart:
            self.__arg = 'on_start'
        elif event == OnExecutionComplete:
            self.__arg = 'on_completion'
        elif event == OnShutdown:
            self.__arg = 'on_shutdown'
        elif event == OnProcessIO:
            if io not in ('stdin', 'stdout', 'stderr'):
                console.error("Cannot guess which sub-event of OnProcessIO is to be treated, specify arg = 'stdin', 'stdout' or 'stderr'")
            self.__arg = 'on_' + io
        else:
            self.__arg = None

    def wrap(self, action):

        if self.__arg is None:
            name = f'{self.__event}'.split()[1][1:-2]
            console.error(f'sorry I cannot handle events of type {name}')

        kwargs = {'target_action': self.__ref,
                  self.__arg: wrap(action, self.__delay)}

        return RegisterEventHandler(self.__event(**kwargs))
