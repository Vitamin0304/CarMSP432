#include "MembershipFuction.h"
namespace fuzzy
{
	TriangleMF::TriangleMF(float a, float b, float c)
		:a(a), b(b), c(c)
	{

	}
	TriangleMF::~TriangleMF()
	{
	}
	float TriangleMF::getValue(float x)
	{
		if (x < a || x > c)
			return 0;
		else if (x < b)
			return (x - a) / (b - a);
		else
			return (x - c) / (b - c);
	}
	float TriangleMF::Integral(float y)
	{
		if (y <= 0 || y > 1)
			return 0;
		else if (y == 1)
			return (c - a) / 2;
		else
			return y * (y / 2 - 1) * (a - c);
	}
	float TriangleMF::IntegralWithX(float y)
	{
		if (y <= 0 || y > 1)
			return 0;
		else if (y == 1)
			return (c - a) * (a + b + c) / 6;
		else
			return -y / 6 * (a - c) * (3 * (a + c) + y * (3 * (b - a - c) + y * (a - 2 * b + c)));
	}

	TrapezoidMF::TrapezoidMF(float a, float b, float c, float d)
		:a(a), b(b), c(c), d(d)
	{

	}
	TrapezoidMF::~TrapezoidMF()
	{
	}
	float TrapezoidMF::getValue(float x)
	{
		if (x < a || x > d)
			return 0;
		else if (x < b)
			return (x - a) / (b - a);
		else if (x <= c)
			return 1;
		else
			return (x - d) / (c - d);
	}
	float TrapezoidMF::Integral(float y)
	{
		if (y <= 0 || y > 1)
			return 0;
		else if (y == 1)
			return (c - b + d - a) / 2;
		else
			return y * (y / 2 * (c - d - b + a) + d - a);
	}
	float TrapezoidMF::IntegralWithX(float y)
	{
		if (y <= 0 || y > 1)
			return 0;
		else
			return y / 2 * (y * (y / 3 * ((c - d) * (c - d) - (b - a) * (b - a)) + d * (c - d) + a * (a - b)) + (d - a) * (d + a));
	}
}
