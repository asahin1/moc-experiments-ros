import numpy as np


class Point2D:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def __eq__(self, other):
        if isinstance(other, Point2D):
            return self.x == other.x and self.y == other.y
        else:
            return False

    def __repr__(self):
        return f"({self.x},{self.y})"

    def __mul__(self, multiplier):
        if isinstance(multiplier, (int, float)):
            return Point2D(multiplier * self.x, multiplier * self.y)
        else:
            raise TypeError(
                "Unsupported operand type(s) for -: 'Point2D' and '{}'".format(
                    type(multiplier).__name__
                )
            )

    def __rmul__(self, multiplier):
        if isinstance(multiplier, (int, float)):
            return Point2D(multiplier * self.x, multiplier * self.y)
        else:
            raise TypeError(
                "Unsupported operand type(s) for -: 'Point2D' and '{}'".format(
                    type(multiplier).__name__
                )
            )

    def getNorm(self):
        return np.sqrt(self.x**2 + self.y**2)

    def getDistance(self, other):
        if isinstance(other, Point2D):
            dx = self.x - other.x
            dy = self.y - other.y
            return np.sqrt(dx**2 + dy**2)
        else:
            raise TypeError(
                "Unsupported argument for getDistance: '{}'".format(
                    type(other).__name__
                )
            )
