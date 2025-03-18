from typing import Tuple

from utils import SingletonMeta

class Settings(metaclass=SingletonMeta):
    coordinate_names: Tuple[str]

    def set_coordinate_names(self, coordinate_names: Tuple[str]):
        self.coordinate_names = coordinate_names
