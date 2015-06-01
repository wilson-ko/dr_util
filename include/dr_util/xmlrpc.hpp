#pragma once

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

		// Very ugly hack to work around missing const access to map.
		// TODO: Remove once ROS updates to newer XmpRpc++ with const accessors.
		XmlRpc::XmlRpcValue & ugly_hack = const_cast<XmlRpc::XmlRpcValue &>(value);

		std::map<std::string, T> result;
		for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator i = ugly_hack.begin(); i != ugly_hack.end(); ++i) {
			result.insert({i->first, fromXmlRpc<T>(i->second)});
		}

		return result;
	}
};

}
