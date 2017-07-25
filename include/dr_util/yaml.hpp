#pragma once
#include <yaml-cpp/yaml.h>

namespace YAML {

template <typename T, std::size_t N>
struct convert<std::array<T, N>> {
	static Node encode(std::array<T, N> const & rhs) {
		Node node(NodeType::Sequence);

		for (auto const & element : rhs) node.push_back(element);

		return node;
	}

	static bool decode(Node const & node, std::array<T, N> & rhs) {
		if (!isNodeValid(node)) return false;

		for (std::size_t i = 0; i < node.size(); ++i) rhs[i] = node[i].as<T>();

		return true;
	}

	private:
	static bool isNodeValid(Node const & node) {
		return node.IsSequence() && node.size() == N;
	}
};

}
