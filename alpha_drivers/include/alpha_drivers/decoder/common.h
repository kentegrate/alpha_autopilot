#pragma once

template <typename T, size_t N>
  char (&_ARRAY_SIZE_HELPER(T (&_arr)[N]))[N];

template <typename T>
char (&_ARRAY_SIZE_HELPER(T (&_arr)[0]))[0];

#define ARRAY_SIZE(_arr) sizeof(_ARRAY_SIZE_HELPER(_arr))
