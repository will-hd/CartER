from collections.abc import Callable
from typing import TypeVar

from commander.network.protocol import Packet

PacketT = TypeVar("PacketT", bound=Packet, covariant=True)
PacketSelector = Callable[[PacketT], bool]
