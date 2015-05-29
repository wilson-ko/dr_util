#include "polynomial.hpp"

#include <XmlRpcValue.h>

#include <cmath>

namespace dr {

namespace {
	/// Convert an XmlRpcValue to a double.
	/**
	 * \return True on success or false on failure.
	 */
	bool fromXmlRpc(
		XmlRpc::XmlRpcValue const & value, ///< XmlRpc value to convert.
		double & output                    ///<[out] Set to the converted value on success. Not modified on failure.
	) {
		if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
			output = XmlRpc::XmlRpcValue(value);
			return true;
		} else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
			output = int(XmlRpc::XmlRpcValue(value));
			return true;
		} else {
			return false;
		}
	}

	/// Convert an XmlRpcValue to a polynomial term.
	/**
	 * \return True on success or false on failure.
	 */
	bool fromXmlRpc(
		XmlRpc::XmlRpcValue const & value, ///< XmlRpc value to convert.
		Polynomial::Term & output          ///<[out] Set to the converted value on success. Not modified on failure.
	) {
		if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) return false;
		if (value.size() != 2) return false;

		Polynomial::Term result;
		if (!fromXmlRpc(value[0], result.coefficient())) return false;
		if (!fromXmlRpc(value[1], result.exponent())) return false;

		output = result;
		return true;
	}

	/// Convert an XmlRpcValue to a polynomial.
	/**
	 * \return True on success or false on failure.
	 */
	bool fromXmlRpc(
		XmlRpc::XmlRpcValue const & value, ///< XmlRpc value to convert.
		Polynomial & output                ///<[out] Set to the converted value on success. Not modified on failure.
	) {
		if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) return false;

		Polynomial result;
		for (int i = 0; i < value.size(); ++i) {
			Polynomial::Term term;
			if (!fromXmlRpc(value[i], term)) return false;
			result.terms.push_back(term);
		}

		output = result;
		return true;
	}

}

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

bool getParam(std::string const & key, Polynomial & result) {
	XmlRpc::XmlRpcValue value;
	if (!ros::param::get(key, value)) return false;
	return fromXmlRpc(value, result);
}

}
