from queue import Queue, Empty
from typing import Any

import threading
import json
import time


class MetricsCollector:
    """A static class responsible for recording data to a file. A queue is used to buffer the data
    so the I/O operations can happen on a separate thread.
    """

    state = {}
    _file_target = ""
    _thread = None
    _running = False

    @classmethod
    def init_state(cls, state: dict, file_target: str, include: bool = False) -> None:
        """Set the initial state of the data. Also starts the file thread.

        Args:
            state (dict): initial state of the data. All other data must follow this format
            file_target (str): target file
            include (bool, optional): write the initial state. Defaults to False.
        """
        cls.state = state.copy()
        cls.state_queue = Queue(maxsize=0)  # infinite queue

        if include:
            cls.state_queue.put_nowait(state.copy())

        cls._file_target = file_target
        cls._start_thread()

    @classmethod
    def reset(cls) -> None:
        """Resets the state of the class. Must be initialised again."""
        if cls._running and cls._thread is not None:
            cls._running = False
            cls._thread.join()

        cls.state = {}
        cls._file_target = ""
        cls._thread = None

    @classmethod
    def update_key(cls, key: str, value: Any) -> None:
        """Updates the instance of state with the specified key and value. The key must exist within the state,
        initialised using init_state().

        Args:
            key (str): dict key
            value (Any): value
        """

        if key in cls.state.keys():
            cls.state[key] = value

    @classmethod
    def save_state(cls) -> None:
        """Push the current state into the Queue"""
        if cls._running:
            state_cp = cls.state.copy()
            cls.state_queue.put_nowait(state_cp)

    @classmethod
    def fetch_key(cls, key: str) -> Any:
        """Get the value of a key if it exists"""
        if cls._running and key in cls.state.keys():
            return cls.state[key]

    @classmethod
    def fetch_state(cls) -> Any:
        """Return the current state"""
        if cls._running:
            return cls.state.copy()

    @classmethod
    def _start_thread(cls) -> None:
        cls._running = True
        cls._thread = threading.Thread(target=cls._thread_target)
        cls._thread.start()

    @classmethod
    def stop_thread(cls) -> None:
        cls._running = False
        cls._thread.join()  # type: ignore

    @classmethod
    def _thread_target(cls) -> None:
        f = open(cls._file_target, "w")

        f.write("[")
        while cls._running:
            try:
                state = cls.state_queue.get(
                    block=True, timeout=0.05
                )  # can't block thread as it will never finish
                serialized_json = json.dumps(state)
                f.write(serialized_json + ",")
            except Empty:
                pass

        f.write("]")  # last json character
        f.close()


if __name__ == "__main__":
    # debug
    target = "test_buffer.txt"

    state = {"timestamp": time.perf_counter(), "value": 0}
    start = time.perf_counter()

    MetricsCollector.init_state(state, target, include=False)

    iterations = 10
    for i in range(1, iterations + 1):
        time.sleep(1)  # 1 second intervals
        # update state
        MetricsCollector.update_key("timestamp", time.perf_counter())
        MetricsCollector.update_key("value", i)
        MetricsCollector.save_state()

    print(f"Took {time.perf_counter() - start} to push.")

    MetricsCollector.stop_thread()
    print(f"Took {time.perf_counter() - start} to join.")
