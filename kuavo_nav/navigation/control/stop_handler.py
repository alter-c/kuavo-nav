import threading


class StopHandler:
    """中断管理模块"""
    def __init__(self):
        self._stop_event = threading.Event()
    
    def request_stop(self):
        """发布停止信号"""
        self._stop_event.set()
    
    def is_stop(self):
        """检查停止信号"""
        return self._stop_event.is_set()
    
    def clear_stop(self):
        """清除停止信号"""
        self._stop_event.clear()