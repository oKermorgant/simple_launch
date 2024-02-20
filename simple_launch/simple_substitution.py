from launch import Substitution
from launch.launch_context import LaunchContext
from typing import Text, List, Tuple
from os.path import sep


def flatten(nested):
    try:
        for i, elem in enumerate(nested):
            while isinstance(elem, (List,Tuple)):
                nested[i:i+1] = elem
                elem = nested[i]
    except IndexError:
        pass
    return [elem for elem in nested if elem is not None]


def is_basic(elem) -> bool:
    return isinstance(elem, (Text, bool, int, float))


def is_empty(elem) -> bool:
    if isinstance(elem, SimpleSubstitution):
        return all([is_empty(sub) for sub in elem.substitutions()])
    return elem is None or elem == ''


class SimpleSubstitution(Substitution):
    '''
    A container for other substitutions that provides concatenation (+) and path building (/) operators
    '''

    def __init__(self, *elems):
        super().__init__()
        self._subs = list(elems)

    def has_elems(self) -> bool:
        return len(self._subs) > 0

    def split_tail(self) -> tuple:
        if not self.has_elems():
            return [],None
        subs = self.substitutions()
        if isinstance(subs[-1], SimpleSubstitution):
            head, tail = subs[-1].split_tail()
            return subs[:-1]+head, tail
        return subs[:-1], subs[-1]

    def substitutions(self) -> List:
        return flatten(self._subs)

    def describe(self) -> Text:
        return '{}'.format(' + '.join(str(sub) if is_basic(sub) else sub.describe() for sub in self.substitutions()))

    def perform(self, context: LaunchContext) -> Text:
        return ''.join(str(sub) if is_basic(sub) else sub.perform(context) for sub in self.substitutions())

    def __add__(self, other) -> 'SimpleSubstitution':
        return SimpleSubstitution(self._subs, other)

    def __iadd__(self, other) -> 'SimpleSubstitution':
        self._subs.append(other)
        return self

    def __radd__(self, other) -> 'SimpleSubstitution':
        return SimpleSubstitution(other, self._subs)

    def __truediv__(self, other) -> 'SimpleSubstitution':
        this_sep = None if is_empty(other) or is_empty(self) else sep
        return SimpleSubstitution(self._subs, this_sep, other)

    def __rtruediv__(self, other) -> 'SimpleSubstitution':
        this_sep = None if is_empty(other) or is_empty(self) else sep
        return SimpleSubstitution(other, this_sep, self._subs)

    def __itruediv__(self, other) -> 'SimpleSubstitution':
        self._subs = (self / other)._subs
        return self
