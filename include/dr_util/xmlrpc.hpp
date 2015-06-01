#pragma once

#include <XmlRpcValue.h>

#include <string>
#include <vector>

namespace dr {

/// Access an XmlRpc value as a boolean.
bool xmlRpcAsBool(XmlRpc::XmlRpcValue const & value);

/// Access an XmlRpc value as an int.
int xmlRpcAsInt(XmlRpc::XmlRpcValue const & value);

/// Access an XmlRpc value as a double.
double xmlRpcAsDouble(XmlRpc::XmlRpcValue const & value);

/// Access an XmlRpc value as a string.
std::string const & xmlRpcAsString(XmlRpc::XmlRpcValue const & value);

/// Get a member of an XmlRpcValue struct.
XmlRpc::XmlRpcValue const & xmlRpcAt(XmlRpc::XmlRpcValue const & value, std::string const & key);

/// Get an iterator to the first member of an XmlRpcValue struct.
XmlRpc::XmlRpcValue::ValueStruct::const_iterator xmlRpcBegin(XmlRpc::XmlRpcValue const & value);

/// Get an end iterator for the members of an XmlRpcValue struct.
XmlRpc::XmlRpcValue::ValueStruct::const_iterator xmlRpcEnd(XmlRpc::XmlRpcValue const & value);

/// Convert an XmlRpcValue::Type to a string.
std::string xmlRpcTypeName(XmlRpc::XmlRpcValue::Type type);

/// Make a runtime error for an unsupported XmlRpcValue::Type.
std::runtime_error makeXmlRpcValueTypeError(XmlRpc::XmlRpcValue::Type type, std::string const & target_type);

/// Struct to convert an XmlRpc value to a T.
/**
 * Must be explicitly specialized for supported types.
 *
 * Specializations must implement a member to do the conversion:
 *   static T convert(XmlRpc::XmlRpcValue const &);
 */
template<typename T>
struct ConvertXmlRpc;

/// Load a value from a XmlRpcValue.
/**
 * Needs static T ConvertXmlRpc<T>::convert(XmlRpc::XmlRpcValue const &) to be defined.
 *
 * \throws Anything ConvertXmlRpc<T>::convert() throws.
 * \return The loaded value.
 */
template<typename T>
T fromXmlRpc(
	XmlRpc::XmlRpcValue const & value ///< The XmlRpcValue to load from.
) {
	return ConvertXmlRpc<T>::convert(value);
}

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
		if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to a vector.");

		std::vector<T> result;
		for (int i = 0; i < value.size(); ++i) {
			result.push_back(ConvertXmlRpc<T>::convert(value));
		}

		return result;
	}
};

template<typename T>
struct ConvertXmlRpc<std::map<std::string, T>> {
	static std::map<std::string, T> convert(XmlRpc::XmlRpcValue const & value) {
		if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to a map.");

		std::map<std::string, T> result;
		for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator i = xmlRpcBegin(value); i != xmlRpcEnd(value); ++i) {
			result.insert({i->first, fromXmlRpc<T>(i->second)});
		}

		return result;
	}
};

}
