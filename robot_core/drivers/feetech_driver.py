"""Placeholder driver for Feetech STS3215 servos over serial bus."""


class FeetechDriver:
    """Read-only placeholder; real serial bus logic will be added later."""

    def __init__(self, config: dict) -> None:
        self.config = config

    def read_positions(self) -> list[float]:
        raise NotImplementedError("Feetech STS3215 driver is not implemented yet.")
