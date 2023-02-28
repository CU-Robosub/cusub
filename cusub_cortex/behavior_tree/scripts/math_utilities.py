#!/usr/bin/env python

class Vector2():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        return Vector2(self.x * other.x, self.y * other.y)

    def normalized(self, speed = 1.0):
        self.length = self.x**2 + self.y**2
        return (Vector2(self.x / self.length * speed, self.y / self.length * speed))

		

class Vector3():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z