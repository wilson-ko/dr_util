#include "xmlrpc.hpp"

namespace dr {

std::string xmlRpcTypeName(XmlRpc::XmlRpcValue::Type type) {
	switch (type) {
		case XmlRpc::XmlRpcValue::TypeArray:    return "array";
		case XmlRpc::XmlRpcValue::TypeBase64:   return "base64";
		case XmlRpc::XmlRpcValue::TypeBoolean:  return "boolean";
		case XmlRpc::XmlRpcValue::TypeDateTime: return "datetime";
		case XmlRpc::XmlRpcValue::TypeDouble:   return "double";
		case XmlRpc::XmlRpcValue::TypeInt:      return "int";
		case XmlRpc::XmlRpcValue::TypeInvalid:  return "invalid";
		case XmlRpc::XmlRpcValue::TypeString:   return "string";
		case XmlRpc::XmlRpcValue::TypeStruct:   return "struct";
	}

	return "unknown (" + std::to_string(type) + ")";
}

bool ConvertXmlRpc<bool>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) return bool(XmlRpc::XmlRpcValue(value));
	if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)     return int(XmlRpc::XmlRpcValue(value));
	throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to boolean.");
}

int ConvertXmlRpc<int>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return int(XmlRpc::XmlRpcValue(value));
	throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to integer.");
}

double ConvertXmlRpc<double>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return double(XmlRpc::XmlRpcValue(value));
	if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)    return int(XmlRpc::XmlRpcValue(value));
	throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to double.");
}

std::string ConvertXmlRpc<std::string>::convert(XmlRpc::XmlRpcValue const & value) {
	if (value.getType() == XmlRpc::XmlRpcValue::TypeString) return std::string(XmlRpc::XmlRpcValue(value));
	throw std::runtime_error("Cannot convert XmlRpc type " + xmlRpcTypeName(value.getType()) + " to string.");
}

}
