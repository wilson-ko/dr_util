#include <dr_param/xmlrpc.hpp>

#include <vector>
#include <tuple>
#include <ostream>

namespace dr {

/// A polynomial.
class Polynomial {
public:
	/// A single term of a polynomial as (coefficient, exponent) tuple.
	struct Term : std::tuple<double, double> {
		using std::tuple<double, double>::tuple;

		double & coefficient()       noexcept { return std::get<0>(*this); }
		double   coefficient() const noexcept { return std::get<0>(*this); }
		double & exponent()          noexcept { return std::get<1>(*this); }
		double   exponent()    const noexcept { return std::get<1>(*this); }

		/// Compute the value of this term for a given input value.
		double y(double x) const noexcept;
	};

	/// The terms of the polynomial.
	std::vector<Term> terms;

	/// Compute the value of the polynomial for a given input value.
	double y(double x) const noexcept;

	/// Calculate the derivative of a polynomial.
	Polynomial derivative() const;
};

/// Output a polynomial to an ostream.
std::ostream & operator<< (std::ostream & stream, Polynomial const & polynomial);

/// Convert an XmlRpc value to a polynomial.
template<> Polynomial::Term fromXmlRpc<Polynomial::Term>(XmlRpc::XmlRpcValue const & value);

/// Convert an XmlRpc value to a polynomial.
template<> Polynomial fromXmlRpc<Polynomial>(XmlRpc::XmlRpcValue const & value);

}
