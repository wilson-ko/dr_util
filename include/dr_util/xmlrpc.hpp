#include <XmlRpcValue.h>

#include <string>
#include <vector>

namespace dr {

std::string xmlRpcTypeName(XmlRpc::XmlRpcValue::Type type);

/// Struct to convert an XmlRpc value to a T.
/**
 * Must be explicitly specialized for supported types.
 *
 * Specializations must implement a member to do the conversion:
 *   static T convert(XmlRpc::XmlRpcValue const &);
 */
template<typename T>
struct ConvertXmlRpc;

template<>
struct ConvertXmlRpc<bool> {
	static bool convert(XmlRpc::XmlRpcValue const & value);
};

template<>
struct ConvertXmlRpc<int> {
	static int convert(XmlRpc::XmlRpcValue const & value);
};

template<>
struct ConvertXmlRpc<double> {
	static double convert(XmlRpc::XmlRpcValue const & value);
};

template<>
struct ConvertXmlRpc<std::string> {
	static std::string convert(XmlRpc::XmlRpcValue const & value);
};

template<typename T>
struct ConvertXmlRpc<std::vector<T>> {
	static std::vector<T> convert(XmlRpc::XmlRpcValue const & value) {
		if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to vector.");

		std::vector<T> result;
		for (int i = 0; i < value.size(); ++i) {
			result.push_back(ConvertXmlRpc<T>::convert(value));
		}

		return result;
	}
};

}
