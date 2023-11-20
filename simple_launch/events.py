from launch.actions import RegisterEventHandler, TimerAction, GroupAction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete, OnProcessIO, OnShutdown
from . import console


def wrap(action, delay):
    if delay is None:
        return [action]
    if isinstance(delay, int):
        delay = float(delay)
    return [TimerAction(period=delay, actions=[action])]


class When:

    def __init__(self,
                 action = None,
                 event = None,
                 delay = None,
                 io = None):
        self.__ref = action
        self.__delay = delay
        self.__event = event

        if event is None:
            return

        if event == OnProcessExit:
            self.__arg = 'on_exit'
        elif event == OnProcessStart:
            self.__arg = 'on_start'
        elif event == OnExecutionComplete:
            self.__arg = 'on_completion'
        elif event == OnShutdown:
            self.__ref = None
            self.__arg = 'on_shutdown'
        elif event == OnProcessIO:
            if io not in ('stdin', 'stdout', 'stderr'):
                console.error("Cannot guess which sub-event of OnProcessIO is to be treated, specify arg = 'stdin', 'stdout' or 'stderr'")
            self.__arg = 'on_' + io
        else:
            self.__arg = None

    def register(self, action):

        if self.__event == OnProcessIO:
            # this one expects Python functions that returns actions
            kwargs = {self.__arg: None}
            if len(action) == 0:
                console.warn('No callback in OnProcessIO block')
            else:
                def callback(event):
                    if self.__delay is not None:
                        import time
                        time.sleep(self.__delay)
                    return GroupAction([cb(event) for cb in action])
                kwargs[self.__arg] = callback
            return RegisterEventHandler(OnProcessIO(**kwargs))

        if self.__event is None:
            return wrap(action, self.__delay)

        if self.__arg is None:
            name = f'{self.__event}'.split()[1][1:-2]
            console.error(f'sorry I cannot handle events of type {name}')

        wrapped = wrap(action, self.__delay)

        kwargs = {self.__arg: wrapped}
        if self.__ref is not None:
            kwargs['target_action'] = self.__ref

        return RegisterEventHandler(self.__event(**kwargs))
