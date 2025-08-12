from abc import abstractmethod


class SingleLineFollowSensor:
    @abstractmethod
    def line_confidence(self) -> float:
        raise NotImplementedError("This method should be implemented by subclasses.")