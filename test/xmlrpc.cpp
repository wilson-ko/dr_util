#include "xmlrpc.hpp"

#include <gtest/gtest.h>

int main(int argc, char ** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(XmlrpcTest, convertBoolean) {
	XmlRpc::XmlRpcValue val_true   = true;
	XmlRpc::XmlRpcValue val_false  = false;
	XmlRpc::XmlRpcValue val_int_0  = 0;
	XmlRpc::XmlRpcValue val_int_1  = 1;
	XmlRpc::XmlRpcValue val_int_42 = 42;
	XmlRpc::XmlRpcValue val_double = 3.14;
	XmlRpc::XmlRpcValue val_string = "Klaatu barada nikto";

	ASSERT_EQ(true,  fromXmlRpc<bool>(val_true));
	ASSERT_EQ(false, fromXmlRpc<bool>(val_false));
	ASSERT_EQ(false, fromXmlRpc<bool>(val_int_0));
	ASSERT_EQ(true,  fromXmlRpc<bool>(val_int_1));
	ASSERT_EQ(true,  fromXmlRpc<bool>(val_int_42));

	ASSERT_THROW(fromXmlRpc<bool>(val_double), std::exception);
	ASSERT_THROW(fromXmlRpc<bool>(val_string), std::exception);
}

TEST(XmlrpcTest, convertInteger) {
	XmlRpc::XmlRpcValue val_bool   = true;
	XmlRpc::XmlRpcValue val_int    = 42;
	XmlRpc::XmlRpcValue val_double = 3.14;
	XmlRpc::XmlRpcValue val_string = "Klaatu barada nikto";

	ASSERT_EQ(42, fromXmlRpc<int>(val_int));

	ASSERT_THROW(fromXmlRpc<int>(val_bool),   std::exception);
	ASSERT_THROW(fromXmlRpc<int>(val_double), std::exception);
	ASSERT_THROW(fromXmlRpc<int>(val_string), std::exception);
}

TEST(XmlrpcTest, convertDouble) {
	XmlRpc::XmlRpcValue val_bool   = true;
	XmlRpc::XmlRpcValue val_int    = 42;
	XmlRpc::XmlRpcValue val_double = 3.14;
	XmlRpc::XmlRpcValue val_string = "Klaatu barada nikto";

	ASSERT_DOUBLE_EQ(3.14, fromXmlRpc<double>(val_double));
	ASSERT_DOUBLE_EQ(42, fromXmlRpc<double>(val_int));

	ASSERT_THROW(fromXmlRpc<double>(val_bool),   std::exception);
	ASSERT_THROW(fromXmlRpc<double>(val_string), std::exception);
}

TEST(XmlrpcTest, convertString) {
	XmlRpc::XmlRpcValue val_bool   = true;
	XmlRpc::XmlRpcValue val_int    = 42;
	XmlRpc::XmlRpcValue val_double = 3.14;
	XmlRpc::XmlRpcValue val_string = "Klaatu barada nikto";

	ASSERT_EQ("Klaatu barada nikto", fromXmlRpc<std::string>(val_string));

	ASSERT_THROW(fromXmlRpc<std::string>(val_bool),   std::exception);
	ASSERT_THROW(fromXmlRpc<std::string>(val_int),    std::exception);
	ASSERT_THROW(fromXmlRpc<std::string>(val_double), std::exception);
}

TEST(XmlrpcTest, convertVectorInt) {
	XmlRpc::XmlRpcValue val;
	val[0] = 7;
	val[1] = 4;
	val[2] = 1;

	std::vector<int> result = fromXmlRpc<std::vector<int>>(val);
	ASSERT_EQ((std::vector<int>{7, 4, 1}), fromXmlRpc<std::vector<int>>(val));
}

TEST(XmlrpcTest, convertVectorString) {
	XmlRpc::XmlRpcValue val;
	val[0] = "klaatu";
	val[1] = "barada";
	val[2] = "nikto";

	ASSERT_EQ((std::vector<std::string>{"klaatu", "barada", "nikto"}), fromXmlRpc<std::vector<std::string>>(val));
}

TEST(XmlrpcTest, convertVectorInvalid) {
	XmlRpc::XmlRpcValue val;
	val[0] = 7;
	val[1] = 4;
	val[2] = 1;

	ASSERT_THROW(fromXmlRpc<std::vector<std::string>>(val), std::exception);
}

TEST(XmlrpcTest, convertMapInt) {
	XmlRpc::XmlRpcValue val;
	val["aap"]  = 0;
	val["noot"] = 1;
	val["mies"] = 2;

	ASSERT_EQ((std::map<std::string, int>{{"aap", 0}, {"noot", 1}, {"mies", 2}}), (fromXmlRpc<std::map<std::string, int>>(val)));
}

TEST(XmlrpcTest, convertMapString) {
	XmlRpc::XmlRpcValue val;
	val["aap"]  = "wim";
	val["noot"] = "zus";
	val["mies"] = "jet";

	ASSERT_EQ((std::map<std::string, std::string>{{"aap", "wim"}, {"noot", "zus"}, {"mies", "jet"}}), (fromXmlRpc<std::map<std::string, std::string>>(val)));
}

TEST(XmlrpcTest, convertMapInvalid) {
	XmlRpc::XmlRpcValue val;
	val["aap"]  = 0;
	val["noot"] = 1;
	val["mies"] = 2;

	ASSERT_THROW((fromXmlRpc<std::map<std::string, std::string>>(val)), std::exception);
}

}
