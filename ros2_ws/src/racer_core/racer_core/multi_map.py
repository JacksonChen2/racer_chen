"""Chunk bookkeeping from RACER's multi-map manager, independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class MapChunk:
    owner: int
    index: int
    addresses: list[int]
    occupancy: list[int]


@dataclass(slots=True)
class MultiMapManager:
    drone_id: int
    drone_count: int
    chunk_size: int = 200
    chunks: dict[int, dict[int, MapChunk]] = field(default_factory=dict)
    pending_addresses: list[int] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.chunks = {drone: {} for drone in range(1, self.drone_count + 1)}

    def append_addresses(self, addresses: list[int], occupancy_lookup) -> list[MapChunk]:
        self.pending_addresses.extend(int(value) for value in addresses)
        emitted: list[MapChunk] = []
        owner_chunks = self.chunks[self.drone_id]
        while len(self.pending_addresses) >= self.chunk_size:
            selected = self.pending_addresses[: self.chunk_size]
            del self.pending_addresses[: self.chunk_size]
            chunk = MapChunk(
                self.drone_id,
                len(owner_chunks) + 1,
                selected,
                [int(occupancy_lookup(address)) for address in selected],
            )
            owner_chunks[chunk.index] = chunk
            emitted.append(chunk)
        return emitted

    def insert(self, chunk: MapChunk) -> bool:
        owner = self.chunks.setdefault(chunk.owner, {})
        if chunk.index in owner:
            return False
        owner[chunk.index] = chunk
        return True

    def index_intervals(self, owner: int) -> list[int]:
        ids = sorted(self.chunks.get(owner, {}))
        if not ids:
            return []
        result, begin, previous = [], ids[0], ids[0]
        for value in ids[1:]:
            if value != previous + 1:
                result.extend((begin, previous))
                begin = value
            previous = value
        result.extend((begin, previous))
        return result

    def missing(self, owner: int, remote_intervals: list[int]) -> list[int]:
        remote: set[int] = set()
        for begin, end in zip(remote_intervals[::2], remote_intervals[1::2]):
            remote.update(range(begin, end + 1))
        return [index for index in sorted(self.chunks.get(owner, {})) if index not in remote]
