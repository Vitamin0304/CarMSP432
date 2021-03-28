#ifndef FUZZYCONTROL_MEMBERSHIPFUCTION_H_
#define FUZZYCONTROL_MEMBERSHIPFUCTION_H_

namespace fuzzy
{
	class MembershipFuction
	{
	public:
		//计算隶属度
		virtual float getValue(float x) = 0;
		//计算用min方法implied后的积分值，输入为隶属度
		virtual float Integral(float y) = 0;
		//计算用min方法implied后的带自变量的积分值，输入为隶属度
		virtual float IntegralWithX(float y) = 0;
	};
	//三角隶属函数
	class TriangleMF : public MembershipFuction
	{
	public:
		TriangleMF(float a, float b, float c);
		~TriangleMF();
		float getValue(float x) override;
		float Integral(float y);
		float IntegralWithX(float y);
	private:
		float a;
		float b;
		float c;
	};
	//梯形隶属函数
	class TrapezoidMF : public MembershipFuction
	{
	public:
		TrapezoidMF(float a, float b, float c, float d);
		~TrapezoidMF();
		float getValue(float x) override;
		float Integral(float y);
		float IntegralWithX(float y);
	private:
		float a;
		float b;
		float c;
		float d;
	};
}

#endif /* FUZZYCONTROL_MEMBERSHIPFUCTION_H_ */
