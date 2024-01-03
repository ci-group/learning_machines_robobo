from typing import Callable


class ExampleProcess:
    """A small example class to have something that does something"""

    logger: Callable[[str], None]

    def __init__(self, logger: Callable[[str], None] = print) -> None:
        self.logger = logger

    def example_method(self, what: str) -> None:
        self.logger(what)
