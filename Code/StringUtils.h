#ifndef __STRING_UTILS_
#define __STRING_UTILS_

#include <iostream>
#include <random>
#include <string>

using namespace std;

inline string generateRandomString(int length) {
    const string chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dist(0, chars.size() - 1);

    string result;
    for (int i = 0; i < length; ++i) {
        result += chars[dist(gen)];
    }
    return result;
}

#endif

