"""Placeholder driver for Damiao motors over USB2CAN."""


class DamiaoDriver:
    """Read-only placeholder; real USB2CAN logic will be added later."""

    def __init__(self, config: dict) -> None:
        self.config = config

    def read_positions(self) -> list[float]:
        raise NotImplementedError("Damiao USB2CAN driver is not implemented yet.")

    def emergency_stop(self) -> None:
        raise NotImplementedError("Damiao emergency stop is not implemented yet.")
