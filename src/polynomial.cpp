#include "polynomial.hpp"

#include <XmlRpcValue.h>

#include <cmath>
#include <string>
#include <stdexcept>

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

Polynomial::Term ConvertXmlRpc<Polynomial::Term>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to polynomial term.");
	if (value.size() != 2) throw std::runtime_error("Wrong number of elements for polynomial term: " + std::to_string(value.size()) + " (expected 2).");

	return Polynomial::Term(ConvertXmlRpc<double>::convert(value[0]), ConvertXmlRpc<double>::convert(value[1]));
}

Polynomial ConvertXmlRpc<Polynomial>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to polynomial.");

	Polynomial result;
	for (int i = 0; i < value.size(); ++i) {
		result.terms.push_back(ConvertXmlRpc<Polynomial::Term>::convert(value[i]));
	}

	return result;
}

}
