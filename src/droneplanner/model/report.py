"""Risk/feasibility report data contract (Software-Spec §5.4).

The report is the human-in-the-loop artifact: the planner emits a plan *plus*
this report and never assumes blind execution. Its top-level field is a
GO / REVIEW / NO-GO recommendation.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

from droneplanner.model.plan import CrossingMethod, FlyabilityClass


class Recommendation(StrEnum):
    GO = "GO"
    REVIEW = "REVIEW"
    NO_GO = "NO-GO"


class Severity(StrEnum):
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"


class Finding(BaseModel):
    """One reviewable observation feeding the recommendation."""

    model_config = ConfigDict(frozen=True)

    severity: Severity
    code: str = Field(description="Stable machine code, e.g. 'fiber_insufficient'.")
    message: str


class SegmentReport(BaseModel):
    model_config = ConfigDict(frozen=True)

    index: int = Field(ge=0)
    length_m: float = Field(ge=0)
    flyability: FlyabilityClass
    exposed: bool


class CrossingReport(BaseModel):
    model_config = ConfigDict(frozen=True)

    method: CrossingMethod
    feasible: bool
    needs_manipulation: bool
    label: str | None = None


class RiskReport(BaseModel):
    """Machine (``report.json``) + human (``report.md``) risk report."""

    model_config = ConfigDict(frozen=True)

    mission_name: str
    route_index: int = Field(default=0, ge=0)
    recommendation: Recommendation
    generated_at: datetime | None = Field(
        default=None, description="Wall-clock time; None in deterministic/test runs."
    )

    total_length_m: float = Field(ge=0)
    fiber_used_m: float = Field(ge=0)
    fiber_available_m: float = Field(ge=0)
    exposed_length_m: float = Field(ge=0)

    non_navigable_count: int = Field(ge=0, default=0)
    marginal_count: int = Field(ge=0, default=0)
    manipulation_crossings: int = Field(ge=0, default=0)

    segments: tuple[SegmentReport, ...] = ()
    crossings: tuple[CrossingReport, ...] = ()
    objective_scores: dict[str, float] = Field(default_factory=dict)
    findings: tuple[Finding, ...] = ()
