//
// Created by Maxim Dobryakov on 13/10/2020.
//

#include "StringExt.h"


#include <cstdarg>

std::string formatString(const char *format, ...) {
    char buffer[256];

    va_list args;
    va_start(args, format);
    int result = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (result < 0)
        abort();

    return std::string(buffer);
}
