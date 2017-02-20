#include "polynomial.hpp"

#include <cmath>

namespace dr {

double Polynomial::Term::y(double x) const noexcept {
	return coefficient() * std::pow(x, exponent());
}

double Polynomial::y(double x) const noexcept {
	double result = 0;
	for (Term const & term : terms) result += term.y(x);
	return result;
}

Polynomial Polynomial::derivative() const {
	Polynomial derived;

	for (Term const & term : terms) {
		if (term.coefficient() == 0 || term.exponent() == 0) continue;
		derived.terms.push_back(Term(term.coefficient() * term.exponent(), term.exponent() - 1));
	}

	return derived;
}

std::ostream & operator<< (std::ostream & stream, Polynomial const & polynomial) {
	for (Polynomial::Term const & term : polynomial.terms) {
		stream << term.coefficient() << "x" << "^" << term.exponent();
		if (&term != &polynomial.terms.back()) stream << " + ";
	}

	return stream;
}

}
