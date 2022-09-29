from launch import Substitution
from launch.substitutions import TextSubstitution
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
    return filter(lambda elem: elem is not None, self.__subs)

def is_basic(elem):
    return isinstance(elem, (Text, bool, int, float))

class SimpleSubstitution(Substitution):
    '''
    A container for other substitutions that provides concatenation (+) and path building (/) operators
    '''

    def __init__(self, *elems):
        super().__init__()
        self.__subs = list(elems)

    def has_elems(self) -> bool:
        return len(self.__subs) > 0

    @staticmethod
    def is_none(elem) -> bool:
        return elem is None or (isinstance(elem, SimpleSubstitution) and elem.__subs and all([sub is None for sub in elem.__subs]))

    def substitutions(self) -> List:
        return flatten(self.__subs)

    def describe(self) -> Text:
        return '{}'.format(' + '.join(str(sub) if is_basic(sub) else sub.describe() for sub in self.substitutions()))

    def perform(self, context: LaunchContext) -> Text:
        return ''.join(str(sub) if is_basic(sub) else sub.perform(context) for sub in self.substitutions())

    def __add__(self, other):
        return SimpleSubstitution([self.__subs, other])

    def __iadd__(self, other):
        self = self + other
        return self

    def __radd__(self, other):
        return SimpleSubstitution([other, self.__subs])

    def __truediv__(self, other):
        if self.is_none(other):
            return self
        return SimpleSubstitution([self.__subs, sep, other])

    def __rtruediv__(self, other):
        if self.is_none(other):
            return self
        return SimpleSubstitution([other, sep, self.__subs])

    def __itruediv__(self, other):
        self = self / other
        return self
