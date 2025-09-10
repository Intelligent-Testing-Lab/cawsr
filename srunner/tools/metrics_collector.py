from queue import Queue, Empty
from typing import Any

import threading
import json
import time
import os
import logging

logger = logging.getLogger("scenario-runner")
logger.propagate = False


class MetricsCollector:
    """A static class responsible for recording data to a file. A queue is used to buffer the data
    so the I/O operations can happen on a separate thread.
    """

    state = {}
    _file_target = ""
    _thread = None
    _running = False
    state_queue = Queue(maxsize=0)
    _state_lock = threading.Lock()
    _flush_freq = 100

    @classmethod
    def init_state(cls, state: dict, file_target: str, include: bool = False) -> None:
        """Set the initial state of the data. Also starts the file thread.

        Args:
            state (dict): initial state of the data. All other data must follow this format
            file_target (str): target file
            include (bool, optional): write the initial state. Defaults to False.
        """
        with cls._state_lock:
            cls.state = state.copy()

        if include:
            cls.state_queue.put_nowait(state.copy())

        cls._file_target = file_target
        cls._start_thread()

    @classmethod
    def reset(cls) -> None:
        """Resets the state of the class. Must be initialised again."""
        if cls._running:
            cls.stop_thread()

        with cls._state_lock:
            cls.state = {}
        cls._file_target = ""
        cls._thread = None
        cls.state_queue = Queue(maxsize=0)

    @classmethod
    def update_key(cls, key: str, value: Any) -> None:
        """Updates the instance of state with the specified key and value. The key must exist within the state,
        initialised using init_state().

        Args:
            key (str): dict key
            value (Any): value
        """
        with cls._state_lock:
            if key in cls.state:
                cls.state[key] = value

    @classmethod
    def save_state(cls) -> None:
        """Push the current state into the Queue"""
        if cls._running:
            with cls._state_lock:
                state_cp = cls.state.copy()
            cls.state_queue.put_nowait(state_cp)

    @classmethod
    def fetch_key(cls, key: str) -> Any:
        """Get the value of a key if it exists"""
        if cls._running:
            with cls._state_lock:
                if key in cls.state:
                    return cls.state[key]
        return None

    @classmethod
    def fetch_state(cls) -> Any:
        """Return the current state"""
        if cls._running:
            with cls._state_lock:
                return cls.state.copy()
        return {}

    @classmethod
    def _start_thread(cls) -> None:
        if not cls._running:
            cls._running = True
            cls._thread = threading.Thread(target=cls._thread_target)
            cls._thread.start()

    @classmethod
    def stop_thread(cls) -> None:
        """Signals the thread to stop and waits for it to finish."""
        if cls._running and cls._thread is not None:
            cls._running = False
            cls._thread.join()

    @classmethod
    def _thread_target(cls) -> None:
        # flush every 100 pushes
        _pos = 0
        try:
            with open(cls._file_target, "w") as f:
                f.write("[")
                is_first_item = True

                while cls._running or not cls.state_queue.empty():
                    try:
                        state = cls.state_queue.get(block=True, timeout=0.05)

                        if not is_first_item:
                            f.write(",")

                        json.dump(state, f)
                        is_first_item = False

                        print("Pushed")
                        _pos += 1

                        if _pos % cls._flush_freq == 0:
                            print("Flushing buffer")
                            start = time.perf_counter()
                            f.flush()
                            os.fsync(f.fileno())
                            print(f"Flushing took {time.perf_counter() - start}ms")
                    except Empty:
                        # Queue was empty, just continue the loop to check cls._running again
                        pass
                f.write("]")
        except Exception as e:
            logger.error(f"Error in MetricsCollector thread: {e}")


if __name__ == "__main__":
    # debug
    target = "test_buffer.txt"

    state = {"timestamp": time.perf_counter(), "value": 0}
    start = time.perf_counter()

    MetricsCollector.init_state(state, target, include=False)

    iterations = 1000
    for i in range(1, iterations + 1):
        time.sleep(0.01)  # 1 second intervals
        # update state
        MetricsCollector.update_key("timestamp", time.perf_counter())
        state = MetricsCollector.fetch_state()
        MetricsCollector.update_key("value", i)
        MetricsCollector.save_state()

    print(f"Took {time.perf_counter() - start} to push.")

    MetricsCollector.stop_thread()
    print(f"Took {time.perf_counter() - start} to join.")
