#pragma once
class Point2f
{
public:	
	float x, y;
	Point2f() : x(0), y(0) {}
	Point2f(float x, float y) : x(x), y(y) {}
	Point2f operator+(const Point2f& p) const
	{
		return Point2f(x + p.x, y + p.y);
	}
	Point2f operator-(const Point2f& p) const
	{
		return Point2f(x - p.x, y - p.y);
	}
	Point2f operator*(float f) const
	{
		return Point2f(x * f, y * f);
	}
	friend Point2f operator*(float f, const Point2f& p)
	{
		return Point2f(p.x * f, p.y * f);
	}
	float operator[](int i) const
	{
		return i == 0 ? x : y;
	}
	float& operator[](int i)
	{
		return i == 0 ? x : y;
	}

};

