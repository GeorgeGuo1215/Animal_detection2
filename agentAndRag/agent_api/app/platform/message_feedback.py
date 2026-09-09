"""Wire-format compatibility for the single nullable boolean feedback column."""
from typing import Literal

Rating = Literal["up", "down"] | None


def rating_to_good(rating: Rating) -> bool | None:
    if rating is None:
        return None
    if rating not in ("up", "down"):
        raise ValueError("invalid message rating")
    return rating == "up"


def good_to_rating(value: bool | None) -> Rating:
    if value is None:
        return None
    if type(value) is not bool:
        raise ValueError("feedback must be boolean or null")
    return "up" if value else "down"
