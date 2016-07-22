#pragma once

#include <type_traits>

#define FUNCTOR_TYPEDEF(name, rettype, ...) \
  typedef Functor<rettype, ## __VA_ARGS__> name

#define FUNCTOR_DECLARE(name, rettype, ...) \
  Functor<rettype, ## __VA_ARGS__> name

#define FUNCTOR_BIND(obj, func, rettype, ...) \
  Functor<rettype, ## __VA_ARGS__>::bind<std::remove_reference<decltype(*obj)>::type, func>(obj)

#define FUNCTOR_BIND_MEMBER(func, rettype, ...) \
  Functor<rettype, ## __VA_ARGS__>::bind<std::remove_reference<decltype(*this)>::type, func>(this)



template <class RetType, class... Args>
class Functor
{
public:
    constexpr Functor(void *obj, RetType (*method)(void *obj, Args...))
        : _obj(obj)
        , _method(method)
    {
    }

    // Allow to construct an empty Functor
    constexpr Functor(decltype(nullptr))
        : Functor(nullptr, nullptr) { }

    constexpr Functor()
        : Functor(nullptr, nullptr) { }

    // Call the method on the obj this Functor is bound to
    RetType operator()(Args... args) const
    {
        return _method(_obj, args...);
    }

    // Compare if the two Functors are calling the same method in the same
    // object
    inline bool operator==(const Functor<RetType, Args...>& rhs)
    {
        return _obj == rhs._obj && _method == rhs._method;
    }

    // Allow to check if there's a method set in the Functor
    explicit operator bool() const
    {
        return _method != nullptr;
    }

    template<class T, RetType (T::*method)(Args...)>
    static constexpr Functor bind(T *obj)
    {
        return { obj, method_wrapper<T, method> };
    }

private:
    void *_obj;
    RetType (*_method)(void *obj, Args...);

    template<class T, RetType (T::*method)(Args...)>
    static RetType method_wrapper(void *obj, Args... args)
    {
        T *t = static_cast<T*>(obj);
        return (t->*method)(args...);
    }
};
