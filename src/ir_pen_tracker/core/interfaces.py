from abc import ABC, abstractmethod
from typing import Optional
from .types import Frame, BrushPoseVis, Skeleton

class ICamera(ABC):
    @abstractmethod
    def open(self) -> bool:
        pass
        
    @abstractmethod
    def read_frame(self) -> Optional[Frame]:
        pass
        
    @abstractmethod
    def close(self):
        pass

class IIMUDriver(ABC):
    @abstractmethod
    def start(self, port: str):
        pass
        
    @abstractmethod
    def get_window(self, t_start: float, t_end: float):
        pass
        
    @abstractmethod
    def stop(self):
        pass

class IBrushTracker(ABC):
    @abstractmethod
    def track(self, frame: Frame) -> BrushPoseVis:
        pass

class IBodyTracker(ABC):
    @abstractmethod
    def track(self, frame: Frame) -> Optional[Skeleton]:
        pass
