from functools import wraps
from multiprocessing import Lock

from typing import (
    Callable,
    Generic,
    Optional,
    TypeVar,
    Union,
    Iterator,
    Set,
    AbstractSet,
)

T = TypeVar("T")
S = TypeVar("S")


# Sadly ParamSpec is a python 3.10 thing.
def locked_method(f: Callable[..., T]) -> Callable[..., T]:
    """Make an object method atomic. Requires a lock object at self._lock"""

    @wraps(f)
    def inner(self, *args, **kwargs):
        with self._lock:
            return f(self, *args, **kwargs)

    return inner


class LockedSet(set, Generic[T]):
    """An implementation of set that is thread-safe.
    Technically unneeded, as the set() in Cpython is already atomic with the GIL,
    but used anyway, as relying on implementation details is not good practice
    """

    def __init__(self, *args, **kwargs):
        self._lock = Lock()
        super().__init__(*args, **kwargs)

    @locked_method
    def add(self, __element: T):
        return super().add(__element)

    @locked_method
    def remove(self, __element: T):
        return super().remove(__element)

    @locked_method
    def discard(self, __element: T) -> None:
        return super().discard(__element)

    @locked_method
    def __contains__(self, __o: object) -> bool:
        return super().__contains__(__o)

    @locked_method
    def __sub__(self, __value: AbstractSet[Optional[T]]) -> Set[T]:
        return super().__sub__(__value)

    @locked_method
    def __and__(self, __value: AbstractSet[object]) -> Set[T]:
        return super().__and__(__value)

    @locked_method
    def __or__(self, __value: AbstractSet[S]) -> Set[Union[T, S]]:
        return super().__or__(__value)

    @locked_method
    def __iter__(self) -> Iterator[T]:
        return super().__iter__()

    @locked_method
    def __len__(self) -> int:
        return super().__len__()
