"""Pure competition-start state machine shared by the serial bridge and tests."""

from __future__ import annotations

from dataclasses import dataclass, field


STANDBY = "STANDBY"
READY = "READY"
RUNNING = "RUNNING"


@dataclass
class CompetitionStateMachine:
    """Accept unique Arduino START sequences and advance exactly two start states."""

    state: str = STANDBY
    _seen_sequences: set[int] = field(default_factory=set)

    def accept_start(self, sequence: int) -> str | None:
        """Return the new state, or ``None`` for duplicate/invalid/late input."""
        sequence = int(sequence)
        if sequence < 1 or sequence in self._seen_sequences or self.state == RUNNING:
            return None
        self._seen_sequences.add(sequence)
        if self.state == STANDBY:
            self.state = READY
        elif self.state == READY:
            self.state = RUNNING
        else:
            return None
        return self.state
