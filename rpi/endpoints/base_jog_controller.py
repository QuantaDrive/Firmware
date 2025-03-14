from abc import ABC, abstractmethod
from typing import Tuple


class BaseJogController(ABC):
    @abstractmethod
    def config_coordinate_names(self, coordinate_names: Tuple[str]):
        pass

    @abstractmethod
    def get_move(self):
        pass