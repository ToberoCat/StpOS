from libstp.logging import debug, info, error, warn

class ClassNameLogger:
    def debug(self, msg: str) -> None:
        """
        Print a debug message.
        
        Args:
            msg: The message to print
        """
        debug(f"[{self.__class__.__name__}]: {msg}")
    
    def info(self, msg: str) -> None:
        """
        Print an info message.
        
        Args:
            msg: The message to print
        """
        info(f"[{self.__class__.__name__}]: {msg}")

    def warn(self, msg: str) -> None:
        """
        Print an info message.

        Args:
            msg: The message to print
        """
        warn(f"[{self.__class__.__name__}]: {msg}")


    def error(self, msg: str) -> None:
        """
        Print an error message.
        
        Args:
            msg: The message to print
        """
        error(f"[{self.__class__.__name__}]: {msg}")
            