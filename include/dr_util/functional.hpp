#pragma once

#include <cstddef>
#include <functional>
#include <tuple>
#include <iostream>

#include "integer_sequence.hpp"

namespace dr {

	namespace impl {
		template<typename F, template<typename...> class Params, typename... Args, std::size_t... I>
		decltype(std::declval<F>()(std::get<I>(std::declval<Params<Args...>>())...))
		_call(F & func, Params<Args...> const & params, index_sequence<I...>) {
			return func(std::get<I>(params)...);
		}

		template<typename F, template<typename...> class Params, typename... Args, std::size_t... I>
		decltype(std::declval<F const>()(std::get<I>(std::declval<Params<Args...>>())...))
		_call(F const & func, Params<Args...> const & params, index_sequence<I...>) {
			return func(std::get<I>(params)...);
		}
	}

	template<typename F, template<typename...> class Params, typename... Args>
	auto call(F & func, Params<Args...> const & params) -> decltype(impl::_call(func, params, index_sequence_for<Args...>{})) {
		return impl::_call(func, params, index_sequence_for<Args...>{});
	}

	template<typename F, template<typename...> class Params, typename... Args>
	auto call(F const & func, Params<Args...> const & params) -> decltype(impl::_call(func, params, index_sequence_for<Args...>{})) {
		return impl::_call(func, params, index_sequence_for<Args...>{});
	}

//	namespace impl {
//		template<template<typename...> class Functors, typename Params, std::size_t... I, typename... FArgs> struct call_many;
//
//		template<template<typename...> class Functors, typename Params, std::size_t... I, typename F0, typename... FTail>
//		struct call_many<Functors, Params, I..., F0, FTail...> {
//			static void call(Functors<F0, FTail...> const & functors, Params const & params, index_sequence<I...>)
//			{
//				call(std::get<0>(functors), params);
//				call_many(std::make_tuple(std::get<I+1>(functors)...), params, index_sequence_for<FTail...>{});
//			}
//		};
//
//		template<template<typename ...> class Functors, typename Params, std::size_t... I>
//		struct call_many<Functors, Params, I...> {
//			static void call_many(Functors<> const &, Params const &, index_sequence<I...>) {}
//		};
//	}
//
//	template<typename Functors, typename Params>
//	void callMany(Functors const & functors, Params const & params) {
//		impl::call_many(functors, params, make_index_sequence<std::tuple_size<Functors>::value - 1>{});
//	}

}
