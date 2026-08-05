from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, ConfigDict


class Animal(BaseModel):
    model_config = ConfigDict(extra="allow")
    animal_id: str
    species: Optional[str] = None
    name: Optional[str] = None
    breed: Optional[str] = None
    sex: Optional[str] = None
    age_months: Optional[int] = None
    weight_kg: Optional[float] = None


class Device(BaseModel):
    model_config = ConfigDict(extra="allow")
    device_id: str
    firmware: Optional[str] = None
    sampling_hz: Optional[dict] = None


class Window(BaseModel):
    model_config = ConfigDict(extra="allow")
    start_ts: Optional[str] = None
    end_ts: Optional[str] = None
    timezone: Optional[str] = None


class Context(BaseModel):
    model_config = ConfigDict(extra="allow")
    notes: Optional[str] = None
    tags: Optional[List[str]] = None
    location: Optional[dict] = None


class AccelSample(BaseModel):
    t_ms: int
    x: float
    y: float
    z: float


class VitalsSample(BaseModel):
    t_s: int
    hr: Optional[float] = None
    rr: Optional[float] = None


class TempSample(BaseModel):
    t_s: int
    value: float


class AccelSignal(BaseModel):
    model_config = ConfigDict(extra="allow")
    samples: List[AccelSample] = []


class VitalsSignal(BaseModel):
    model_config = ConfigDict(extra="allow")
    samples: List[VitalsSample] = []


class TempSignal(BaseModel):
    model_config = ConfigDict(extra="allow")
    samples: List[TempSample] = []


class Signals(BaseModel):
    model_config = ConfigDict(extra="allow")
    accel: Optional[AccelSignal] = None
    vitals: Optional[VitalsSignal] = None
    temperature: Optional[TempSignal] = None


class Event(BaseModel):
    model_config = ConfigDict(extra="allow")
    event_id: str
    ts: str
    animal: Animal
    device: Optional[Device] = None
    window: Optional[Window] = None
    context: Optional[Context] = None
    signals: Optional[Signals] = None


def normalize_event(event: Event) -> dict:
    return event.model_dump()
