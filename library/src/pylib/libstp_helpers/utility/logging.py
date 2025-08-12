from functools import wraps
from libstp.logging import debug

def log(func):
    """Decorator to log method/function calls with arguments."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        func_name = func.__qualname__
        debug(f"[CallLog:Custom] {func_name}: ({args[1:] if len(args) > 1 else args}, {kwargs})")
        return func(*args, **kwargs)
    return wrapper

class DeviceLogger:
    def __init__(self, native_instance):
        self._native = native_instance
        self.ignore = [
            "get_current_heading",
            "__apply_kinematics_model__"
        ]

    def __getattr__(self, name):
        attr = getattr(self._native, name)
        if callable(attr) and name not in self.ignore:
            @wraps(attr)
            def wrapper(*args, **kwargs):
                debug(f"[CallLog:Device] Called {name}: ({args}, {kwargs})")
                return attr(*args, **kwargs)
            return wrapper
        return attr

def log_method_call(func):
    """Deprecated: use @log instead."""
    return log(func)

class MethodLoggerMeta(type):
    def __new__(cls, name, bases, dct):
        for attr_name, attr in dct.items():
            if callable(attr) and not attr_name.startswith("__"):
                dct[attr_name] = log(attr)
        return super().__new__(cls, name, bases, dct)