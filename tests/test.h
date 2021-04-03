#pragma once
#include "include/common/common.h"
#include "string"
#include "fmt/format.h"
#include "fmt/core.h"

namespace Physics2D
{
	class Test
	{
	public:
		Test(){}
		Test(const std::string& name): m_name(name)
		{}
		virtual void run() = 0;
		void test()
		{
			fmt::print("-----{} starts-----\n", m_name);
			run();
			fmt::print("-----{} ends-----\n", m_name);
		}
	protected:
		std::string m_name;
	};
}
