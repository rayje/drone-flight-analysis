"""Read-only PX4 ULog adapter."""

from pathlib import Path

from pyulog import ULog

from .models import Topic


def read_ulog(path: str | Path) -> tuple[Topic, ...]:
    """Read a ULog without changing it, retaining topic instances and fields."""
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(source)
    log = ULog(str(source))
    return tuple(
        Topic(dataset.name, dataset.multi_id,
              {name: tuple(values) for name, values in dataset.data.items()})
        for dataset in log.data_list
    )
