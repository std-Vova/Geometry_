#include <iostream>
#include "Geometry.h"

bool Shape::operator==(const Shape& other) const {
    try {
        if (typeid(*this) == typeid(other)) {
            return true;
        }
    }
    catch (std::bad_typeid& error) {
        std::cout << error.what();
        throw;  // for test
        return false;
    }
    return false;
}