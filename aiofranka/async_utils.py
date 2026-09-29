"""Utilities for running blocking functions without starving the 1kHz control loop.

After controller.start(), the asyncio event loop must not be blocked for more
than ~1ms.  These helpers offload synchronous work to a thread executor so the
real-time loop keeps running.

Usage::

    from aiofranka import asyncify

    # Decorator
    class MyPolicy:
        @asyncify
        def get_action(self, obs):
            return self.model(obs)

    action = await policy.get_action(obs)

    # Wrap an existing function
    model_async = asyncify(model)
    result = await model_async(input_tensor)

    # Original sync function is accessible via .sync
    result = policy.get_action.sync(obs)
"""

import asyncio
import functools
import multiprocessing
import queue
import threading


def asyncify(fn):
    """Wrap a blocking function so it runs in a thread executor when awaited.

    Works as a decorator or as a wrapper for existing functions::

        @asyncify
        def heavy_compute(x):
            return x ** 2

        result = await heavy_compute(42)
        result = heavy_compute.sync(42)  # original sync version
    """
    @functools.wraps(fn)
    async def wrapper(*args, **kwargs):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, functools.partial(fn, *args, **kwargs))

    wrapper.sync = fn
    return wrapper


class CudaInferenceThread:
    """A persistent thread for CUDA inference that avoids run_in_executor overhead.

    run_in_executor dispatches to a thread pool, which causes ~40x slowdown for
    CUDA ops due to thread-pool dispatch and CUDA cross-thread synchronization.
    This class keeps a single dedicated thread alive with an initialized CUDA
    context, so the per-call cost is just a queue round-trip (~microseconds).

    Usage::

        infer = CudaInferenceThread()
        infer.start()

        # wrap any callable
        action = await infer.run(policy.get_action, obs)

        # or use as a decorator
        @infer.wrap
        def get_action(obs):
            return model(obs)

        action = await get_action(obs)
        action = get_action.sync(obs)  # original sync version

        infer.stop()
    """

    def __init__(self):
        self._req_queue = queue.Queue()
        self._thread = None

    def start(self):
        """Start the inference thread."""
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Signal the thread to exit."""
        self._req_queue.put(None)

    def _loop(self):
        while True:
            item = self._req_queue.get()
            if item is None:
                break
            fn, args, kwargs, loop, future = item
            try:
                result = fn(*args, **kwargs)
                loop.call_soon_threadsafe(future.set_result, result)
            except BaseException as e:
                loop.call_soon_threadsafe(future.set_exception, e)

    async def run(self, fn, *args, **kwargs):
        """Submit a callable and await the result."""
        loop = asyncio.get_event_loop()
        future = loop.create_future()
        self._req_queue.put((fn, args, kwargs, loop, future))
        return await future

    def wrap(self, fn):
        """Decorator: like asyncify but runs on the persistent CUDA thread."""
        @functools.wraps(fn)
        async def wrapper(*args, **kwargs):
            return await self.run(fn, *args, **kwargs)

        wrapper.sync = fn
        return wrapper


class _RemoteAttr:
    """Returned by ProcessProxy.__getattr__. Awaitable for attribute access,
    callable for method calls.

        await proxy.some_attr          # attribute access
        await proxy.some_method(args)  # method call
    """

    def __init__(self, proxy, name):
        self._proxy = proxy
        self._name = name

    def __call__(self, *args, **kwargs):
        return self._proxy._send("call", self._name, args, kwargs)

    def __await__(self):
        return self._proxy._send("getattr", self._name, (), {}).__await__()


class ProcessProxy:
    """Transparent proxy to an object living in a separate process.

    All attribute access and method calls are forwarded to the child process
    via a pipe. This completely avoids GIL contention — the child has its own
    GIL, so a 1kHz control loop on the main process cannot interfere.

    Created via ``mpify()``, not directly.
    """

    def __init__(self, conn, process):
        object.__setattr__(self, "_conn", conn)
        object.__setattr__(self, "_process", process)

    def __getattr__(self, name):
        return _RemoteAttr(self, name)

    async def _send(self, msg_type, method_name, args, kwargs):
        conn = object.__getattribute__(self, "_conn")
        loop = asyncio.get_event_loop()
        conn.send((msg_type, method_name, args, kwargs))
        # recv() blocks but releases the GIL (it's I/O), so run_in_executor is fine
        status, value = await loop.run_in_executor(None, conn.recv)
        if status == "ok":
            return value
        raise RuntimeError(value)

    def stop(self):
        """Signal the child process to exit."""
        conn = object.__getattribute__(self, "_conn")
        process = object.__getattribute__(self, "_process")
        conn.send(None)
        process.join(timeout=5)


def _mp_worker(conn, factory_fn, factory_args, factory_kwargs):
    try:
        obj = factory_fn(*factory_args, **factory_kwargs)
    except Exception as e:
        conn.send(f"error: {e}")
        return
    conn.send("ready")
    while True:
        item = conn.recv()
        if item is None:
            break
        msg_type, method_name, args, kwargs = item
        try:
            if msg_type == "getattr":
                result = getattr(obj, method_name)
            else:
                result = getattr(obj, method_name)(*args, **kwargs)
            conn.send(("ok", result))
        except Exception as e:
            conn.send(("error", str(e)))


def mpify(factory_fn, *args, **kwargs):
    """Spawn a child process, run ``factory_fn(*args, **kwargs)`` in it, and
    return a transparent async proxy to the created object.

    The proxy forwards all method calls and attribute access to the child
    process. Use ``await`` on every access since it crosses a process boundary.

    Usage::

        def make_policy(checkpoint, device):
            model, config = load_model(checkpoint, device)
            return ACTInferencePolicy(model, config, device)

        policy = mpify(make_policy, checkpoint, "cuda:0")

        await policy.reset(initial_ee)
        ee = await policy.get_action(ee, qpos, task_type)
        timing = await policy._last_timing

        policy.stop()  # clean shutdown (synchronous)

    Args:
        factory_fn: Callable that creates the object. Must be picklable
            (module-level function). Runs in the child process.
        *args: Passed to factory_fn.
        **kwargs: Passed to factory_fn.

    Returns:
        ProcessProxy that forwards attribute/method access to the child.
    """
    ctx = multiprocessing.get_context("spawn")
    parent_conn, child_conn = ctx.Pipe()
    process = ctx.Process(
        target=_mp_worker,
        args=(child_conn, factory_fn, args, kwargs),
        daemon=True,
    )
    process.start()
    child_conn.close()
    msg = parent_conn.recv()
    if msg != "ready":
        raise RuntimeError(f"Child process failed to start: {msg}")
    return ProcessProxy(parent_conn, process)


async def async_input(prompt: str = "") -> str:
    """Async-safe replacement for built-in ``input()``.

    Runs ``input()`` in a thread executor so the event loop (and the 1kHz
    control loop) keeps running while waiting for user input::

        await async_input("Press Enter to start...")
    """
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, input, prompt)
