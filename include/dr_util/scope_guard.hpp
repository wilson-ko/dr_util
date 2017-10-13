#pragma once
#include <utility>

namespace dr {

template<typename F>
class ScopeGuard {
private:
	F f;
	bool valid = true;

public:
	ScopeGuard(F f) : f(std::move(f)) {}

	ScopeGuard(ScopeGuard && other): f{std::move(other.f)} {
		other.reset();
	}

	ScopeGuard & operator= (ScopeGuard && other) {
		f = std::move(other.f);
		other.reset();
	}

	~ScopeGuard() {
		if (valid) f();
		valid = false;
	}

	void reset() {
		valid = false;
	}
};

template<typename F>
ScopeGuard<F> scopeGuard(F f) {
	return {std::move(f)};
}

}
