from abc import ABC, abstractmethod

class BaseProgramEndpoint(ABC):
    @abstractmethod
    def start(self):
        pass